/*
 * SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "umd/device/tt_device/protocol/pcie_protocol.hpp"

#include <fmt/base.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <tt-logger/tt-logger.hpp>
#include <utility>
#include <variant>

#include "umd/device/arch/architecture_implementation.hpp"
#include "umd/device/pcie/pci_device.hpp"
#include "umd/device/pcie/silicon_tlb_window.hpp"
#include "umd/device/pcie/tlb_window.hpp"
#include "umd/device/tt_device/protocol/pcie_dma/blackhole_dma_transfer.hpp"
#include "umd/device/tt_device/protocol/pcie_dma/wormhole_dma_transfer.hpp"
#include "umd/device/types/arch.hpp"
#include "umd/device/types/tlb.hpp"
#include "utils.hpp"

namespace tt::umd {

DmaTransferStrategy PcieProtocol::create_dma_strategy(tt::ARCH arch) {
    switch (arch) {
        case tt::ARCH::WORMHOLE_B0:
            return WormholeDmaTransfer{};
        case tt::ARCH::BLACKHOLE:
            return BlackholeDmaTransfer{};
        default:
            UMD_THROW(error::RuntimeError, "Unsupported architecture for DMA transfer strategy.");
    }
}

size_t PcieProtocol::get_dma_tlb_size(tt::ARCH arch) {
    switch (arch) {
        case tt::ARCH::BLACKHOLE:
            return 2 * 1024 * 1024;
        case tt::ARCH::WORMHOLE_B0:
            return 16 * 1024 * 1024;
        default:
            UMD_THROW(error::RuntimeError, "Unsupported architecture for DMA TLB size.");
    }
}

PcieProtocol::PcieProtocol(std::unique_ptr<PCIDevice> pci_device, bool use_safe_api) :
    pci_device_(std::move(pci_device)),
    dma_strategy_(create_dma_strategy(pci_device_->get_arch())),
    use_safe_api_(use_safe_api) {}

PcieProtocol::~PcieProtocol() = default;

void PcieProtocol::set_io_timeout_callback(const std::function<bool(NocId)>& hang_check) {
    hang_check_ = hang_check;
    // The cached window may already exist if I/O ran before the hang detector was wired; keep it in sync.
    if (cached_tlb_window_ != nullptr) {
        cached_tlb_window_->set_io_timeout_hang_check(hang_check_);
    }
}

TlbWindow* PcieProtocol::get_cached_tlb_window() {
    if (cached_tlb_window_ == nullptr) {
        cached_tlb_window_ = std::make_unique<SiliconTlbWindow>(pci_device_->allocate_tlb(
            pci_device_->get_architecture_implementation()->get_cached_tlb_size(), TlbMapping::UC));
        cached_tlb_window_->set_io_timeout_hang_check(hang_check_);
    }
    return cached_tlb_window_.get();
}

void PcieProtocol::write_to_device(const void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    std::lock_guard<std::mutex> lock(io_lock_);
    if (use_safe_api_) {
        write_to_device_impl<true>(mem_ptr, core, addr, size, noc_id);
    } else {
        write_to_device_impl<false>(mem_ptr, core, addr, size, noc_id);
    }
}

void PcieProtocol::read_from_device(void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    std::lock_guard<std::mutex> lock(io_lock_);
    if (use_safe_api_) {
        read_from_device_impl<true>(mem_ptr, core, addr, size, noc_id);
    } else {
        read_from_device_impl<false>(mem_ptr, core, addr, size, noc_id);
    }
}

template <bool safe>
void PcieProtocol::write_to_device_impl(
    const void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    // The cached window carries the per-op MMIO timeout veto (built from its configured NOC + the wired
    // hang check); see SiliconTlbWindow. The reconfigure call sets the NOC before the transfer runs.
    if constexpr (safe) {
        get_cached_tlb_window()->safe_write_block_reconfigure(mem_ptr, core, addr, size, noc_id);
    } else {
        get_cached_tlb_window()->write_block_reconfigure(mem_ptr, core, addr, size, noc_id);
    }
}

template <bool safe>
void PcieProtocol::read_from_device_impl(void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    if constexpr (safe) {
        get_cached_tlb_window()->safe_read_block_reconfigure(mem_ptr, core, addr, size, noc_id);
    } else {
        get_cached_tlb_window()->read_block_reconfigure(mem_ptr, core, addr, size, noc_id);
    }
}

void PcieProtocol::write_to_device_reg(const void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    validate_register_access(addr, size);
    std::lock_guard<std::mutex> lock(io_lock_);
    if (use_safe_api_) {
        get_cached_tlb_window()->safe_write_register_reconfigure(mem_ptr, core, addr, size, noc_id);
    } else {
        get_cached_tlb_window()->write_register_reconfigure(mem_ptr, core, addr, size, noc_id);
    }
}

void PcieProtocol::read_from_device_reg(void* mem_ptr, tt_xy_pair core, uint64_t addr, size_t size, NocId noc_id) {
    validate_register_access(addr, size);
    std::lock_guard<std::mutex> lock(io_lock_);
    if (use_safe_api_) {
        get_cached_tlb_window()->safe_read_register_reconfigure(mem_ptr, core, addr, size, noc_id);
    } else {
        get_cached_tlb_window()->read_register_reconfigure(mem_ptr, core, addr, size, noc_id);
    }
}

bool PcieProtocol::write_to_core_range(
    const void* mem_ptr, tt_xy_pair core_start, tt_xy_pair core_end, uint64_t addr, uint32_t size, NocId noc_id) {
    noc_multicast_write(mem_ptr, size, core_start, core_end, addr, noc_id);
    return true;
}

int PcieProtocol::get_mmio_id() { return pci_device_->get_pci_device_id(); }

void PcieProtocol::noc_multicast_write(
    const void* src, size_t size, tt_xy_pair core_start, tt_xy_pair core_end, uint64_t addr, NocId noc_id) {
    std::lock_guard<std::mutex> lock(io_lock_);
    if (use_safe_api_) {
        get_cached_tlb_window()->safe_noc_multicast_write_reconfigure(
            src, size, core_start, core_end, addr, noc_id, tlb_data::Strict);
    } else {
        get_cached_tlb_window()->noc_multicast_write_reconfigure(
            src, size, core_start, core_end, addr, noc_id, tlb_data::Strict);
    }
}

void PcieProtocol::bar_write32(uint32_t addr, uint32_t data) {
    if (addr < BAR0_OFFSET) {
        UMD_THROW(error::RuntimeError, "Write Invalid BAR address for this device.");
    }
    addr -= BAR0_OFFSET;
    *reinterpret_cast<volatile uint32_t*>(static_cast<uint8_t*>(pci_device_->bar0) + addr) = data;
}

uint32_t PcieProtocol::bar_read32(uint32_t addr) {
    if (addr < BAR0_OFFSET) {
        UMD_THROW(error::RuntimeError, "Read Invalid BAR address for this device.");
    }
    addr -= BAR0_OFFSET;
    return *reinterpret_cast<volatile uint32_t*>(static_cast<uint8_t*>(pci_device_->bar0) + addr);
}

PCIDevice* PcieProtocol::get_pci_device() { return pci_device_.get(); }

bool PcieProtocol::dma_write_to_device(const void* src, size_t size, tt_xy_pair core, uint64_t addr, NocId noc_id) {
    // const_cast is safe here: dma_transfer only reads from the buffer in H2D direction (memcpy into DMA buffer).
    // dma_transfer uses void* to handle both H2D (read) and D2H (write) in a single function.
    // TODO: Split dma_transfer into separate H2D/D2H functions to remove this cast.
    return dma_transfer(
        const_cast<void*>(src), size, addr, create_dma_tlb_config(addr, core, noc_id), DmaDirection::H2D);  // NOLINT
}

bool PcieProtocol::dma_read_from_device(void* dst, size_t size, tt_xy_pair core, uint64_t addr, NocId noc_id) {
    return dma_transfer(dst, size, addr, create_dma_tlb_config(addr, core, noc_id), DmaDirection::D2H);
}

bool PcieProtocol::dma_multicast_write(
    void* src, size_t size, tt_xy_pair core_start, tt_xy_pair core_end, uint64_t addr, NocId noc_id) {
    return dma_transfer(src, size, addr, create_dma_tlb_config(addr, core_end, noc_id, core_start), DmaDirection::H2D);
}

// Creates a TLB config for DMA transfers. Parameters are named core_end/core_start to match
// the x_end/y_end and x_start/y_start fields in tlb_data. For unicast, only core_end is needed
// (the target core). When core_start is provided, the transfer becomes a multicast to the
// core range [core_start, core_end].
tlb_data PcieProtocol::create_dma_tlb_config(
    uint64_t addr, tt_xy_pair core_end, NocId noc_id, std::optional<tt_xy_pair> core_start) {
    tlb_data config{};
    config.local_offset = addr;
    config.x_end = core_end.x;
    config.y_end = core_end.y;
    config.noc_sel = static_cast<uint64_t>(noc_id);
    config.ordering = tlb_data::Relaxed;
    config.static_vc = pci_device_->get_architecture_implementation()->get_static_vc();
    if (core_start) {
        config.x_start = core_start->x;
        config.y_start = core_start->y;
        config.mcast = true;
    }
    return config;
}

bool PcieProtocol::dma_transfer(void* buffer, size_t size, uint64_t addr, tlb_data config, DmaDirection direction) {
    std::scoped_lock lock(dma_mutex_);
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();

    if (dma_buffer.buffer == nullptr) {
        log_warning(LogUMD, "DMA buffer was not allocated for PCI device {}.", pci_device_->get_device_num());
        return false;
    }

    uint8_t* buf = static_cast<uint8_t*>(buffer);
    size_t dmabuf_size = dma_buffer.size;
    TlbWindow* tlb_window = get_cached_dma_tlb_window(config);

    auto axi_address_base = pci_device_->get_architecture_implementation()
                                ->get_tlb_configuration(tlb_window->handle_ref().get_tlb_id())
                                .tlb_offset;

    const size_t tlb_handle_size = tlb_window->handle_ref().get_size();
    auto axi_address = axi_address_base + (addr - (addr & ~(tlb_handle_size - 1)));

    while (size > 0) {
        auto tlb_size = tlb_window->get_size();
        size_t transfer_size = std::min({size, tlb_size, dmabuf_size});

        if (direction == DmaDirection::H2D) {
            std::memcpy(dma_buffer.buffer, buf, transfer_size);
            dma_h2d_transfer(static_cast<uint32_t>(axi_address), dma_buffer.buffer_pa, transfer_size);
        } else {
            const char* e = std::getenv("TT_D2H_CHANNELS");
            if (e) {  // experimental multi-channel path (also used for N=1 to get engine-only timing)
                uint32_t num_channels =
                    std::clamp<uint32_t>(static_cast<uint32_t>(std::strtoul(e, nullptr, 0)), 1u, 8u);
                dma_d2h_multichannel(
                    dma_buffer.buffer_pa, static_cast<uint32_t>(axi_address), transfer_size, num_channels);
            } else {
                dma_d2h_transfer(dma_buffer.buffer_pa, static_cast<uint32_t>(axi_address), transfer_size);
            }
            std::memcpy(buf, dma_buffer.buffer, transfer_size);
        }

        size -= transfer_size;
        addr += transfer_size;
        buf += transfer_size;

        config.local_offset = addr;
        tlb_window->configure(config);
        axi_address = axi_address_base + (addr - (addr & ~(tlb_handle_size - 1)));
    }

    return true;
}

TlbWindow* PcieProtocol::get_cached_dma_tlb_window(tlb_data config) {
    if (cached_dma_tlb_window_ == nullptr) {
        cached_dma_tlb_window_ = std::make_unique<SiliconTlbWindow>(
            pci_device_->allocate_tlb(get_dma_tlb_size(pci_device_->get_arch()), TlbMapping::WC), config);
        return cached_dma_tlb_window_.get();
    }

    cached_dma_tlb_window_->configure(config);
    return cached_dma_tlb_window_.get();
}

// TODO: These public DMA methods are locked for safety since they can be called directly by
// consumers. The goal is to make the protocol class lockless and push synchronization to
// higher-level components. dma_transfer() calls the private _transfer methods directly to
// avoid lock contention.
void PcieProtocol::dma_d2h(void* dst, uint32_t src, size_t size) {
    std::scoped_lock lock(dma_mutex_);
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();

    if (size > dma_buffer.size) {
        UMD_THROW(error::RuntimeError, "DMA size exceeds buffer size.");
    }

    dma_d2h_transfer(dma_buffer.buffer_pa, src, size);
    std::memcpy(dst, dma_buffer.buffer, size);
}

void PcieProtocol::dma_d2h_zero_copy(void* dst, uint32_t src, size_t size) {
    std::scoped_lock lock(dma_mutex_);
    dma_d2h_transfer(reinterpret_cast<uint64_t>(dst), src, size);
}

void PcieProtocol::dma_h2d(uint32_t dst, const void* src, size_t size) {
    std::scoped_lock lock(dma_mutex_);
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();

    if (size > dma_buffer.size) {
        UMD_THROW(error::RuntimeError, "DMA size exceeds buffer size.");
    }

    std::memcpy(dma_buffer.buffer, src, size);
    dma_h2d_transfer(dst, dma_buffer.buffer_pa, size);
}

void PcieProtocol::dma_h2d_zero_copy(uint32_t dst, const void* src, size_t size) {
    std::scoped_lock lock(dma_mutex_);
    dma_h2d_transfer(dst, reinterpret_cast<uint64_t>(src), size);
}

void PcieProtocol::dma_d2h_transfer(const uint64_t dst, const uint32_t src, const size_t size) {
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();
    volatile uint8_t* bar2 = reinterpret_cast<volatile uint8_t*>(pci_device_->bar2_uc);

    if (!dma_buffer.completion || !dma_buffer.buffer) {
        UMD_THROW(error::RuntimeError, "DMA buffer is not initialized.");
    }

    if (src % 4 != 0) {
        UMD_THROW(error::RuntimeError, "DMA source address must be aligned to 4 bytes.");
    }

    if (size % 4 != 0) {
        UMD_THROW(error::RuntimeError, "DMA size must be a multiple of 4.");
    }

    if (!bar2) {
        UMD_THROW(error::RuntimeError, "BAR2 is not mapped.");
    }

    std::visit([&](auto& strategy) { strategy.d2h_transfer(bar2, dma_buffer, dst, src, size); }, dma_strategy_);
}

// Blackhole multi-channel D2H: split [src_axi, +size) across num_channels even (write) channels, each writing
// its slice into a contiguous region of the host staging buffer, then wait on per-channel completion flags.
// The BH controller has 16 channel blocks (BAR2 stride 0x100) = 8 write (even) + 8 read (odd); see
// blackhole_dma_transfer.cpp. Register offsets within a channel: +0x00 EN, +0x04 DOORBELL, +0x1C XFERSIZE,
// +0x20/24 SAR, +0x28/2C DAR, +0x88 INT_SETUP, +0x90/94 MSI_STOP, +0xA0/A4 MSI_ABORT, +0xA8 MSGD.
void PcieProtocol::dma_d2h_multichannel(uint64_t dst_pa, uint32_t src_axi, size_t size, uint32_t num_channels) {
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();
    volatile uint8_t* bar2 = reinterpret_cast<volatile uint8_t*>(pci_device_->bar2_uc);
    if (!dma_buffer.completion || !dma_buffer.buffer || !bar2) {
        UMD_THROW(error::RuntimeError, "DMA buffer/BAR2 not initialized.");
    }
    static constexpr uint32_t DMA_COMPLETION_VALUE = 0xfaca;
    static constexpr uint32_t DMA_TIMEOUT_MS = 10000;
    auto wr = [&](uint32_t off, uint32_t v) { *reinterpret_cast<volatile uint32_t*>(bar2 + off) = v; };

    // Uniform per-channel slice (4-byte aligned); the last channel absorbs the remainder.
    size_t slice = (size / num_channels) & ~size_t(3);
    if (slice == 0) {  // too small to split — fall back to single channel
        dma_d2h_transfer(dst_pa, src_axi, size);
        return;
    }

    auto completion_ptr = [&](uint32_t i) {
        return reinterpret_cast<volatile uint32_t*>(dma_buffer.completion + i * 64);
    };

    // Phase 1: program every channel (no doorbell yet) so they can all launch back-to-back.
    for (uint32_t i = 0; i < num_channels; i++) {
        uint32_t chbase = i * 0x200;  // even (write) channels: 0x000, 0x200, ... 0xE00
        size_t off = static_cast<size_t>(i) * slice;
        size_t this_size = (i == num_channels - 1) ? (size - off) : slice;
        uint64_t comp_pa = dma_buffer.completion_pa + i * 64;
        *completion_ptr(i) = 0;
        wr(chbase + 0x88, 0x28);  // INT_SETUP
        wr(chbase + 0x90, static_cast<uint32_t>(comp_pa & 0xFFFFFFFF));
        wr(chbase + 0x94, static_cast<uint32_t>((comp_pa >> 32) & 0xFFFFFFFF));
        wr(chbase + 0xA0, static_cast<uint32_t>((comp_pa + 4) & 0xFFFFFFFF));
        wr(chbase + 0xA4, static_cast<uint32_t>(((comp_pa + 4) >> 32) & 0xFFFFFFFF));
        wr(chbase + 0xA8, DMA_COMPLETION_VALUE);
        wr(chbase + 0x00, 0x1);                                              // EN
        wr(chbase + 0x20, src_axi + static_cast<uint32_t>(off));             // SAR = device source
        wr(chbase + 0x24, 0);
        wr(chbase + 0x28, static_cast<uint32_t>((dst_pa + off) & 0xFFFFFFFF));  // DAR = host staging slice
        wr(chbase + 0x2C, static_cast<uint32_t>(((dst_pa + off) >> 32) & 0xFFFFFFFF));
        wr(chbase + 0x1C, static_cast<uint32_t>(this_size));                 // XFERSIZE
    }
    // Phase 2: ring all doorbells (channels run concurrently from here).
    auto t0 = std::chrono::steady_clock::now();
    for (uint32_t i = 0; i < num_channels; i++) {
        wr(i * 0x200 + 0x04, 0x1);
    }
    // Phase 3: wait on every channel's host completion flag.
    for (uint32_t i = 0; i < num_channels; i++) {
        while (*completion_ptr(i) != DMA_COMPLETION_VALUE) {
            auto el = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
            if (el > DMA_TIMEOUT_MS) {
                UMD_THROW(error::RuntimeError, "Multi-channel D2H DMA timeout.");
            }
        }
    }
    if (std::getenv("TT_D2H_DEBUG")) {
        double us = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count() /
                    1000.0;
        fmt::print(stderr, "[d2h-multi] {} ch, {} bytes, engine-only {:.1f}us = {:.2f} GB/s\n",
                   num_channels, size, us, static_cast<double>(size) / (us * 1e3));
    }
}

void PcieProtocol::dma_h2d_transfer(const uint32_t dst, const uint64_t src, const size_t size) {
    DmaBuffer& dma_buffer = pci_device_->get_dma_buffer();
    volatile uint8_t* bar2 = reinterpret_cast<volatile uint8_t*>(pci_device_->bar2_uc);

    if (!dma_buffer.completion || !dma_buffer.buffer) {
        UMD_THROW(error::RuntimeError, "DMA buffer is not initialized.");
    }

    if (dst % 4 != 0) {
        UMD_THROW(error::RuntimeError, "DMA destination address must be aligned to 4 bytes.");
    }

    if (size % 4 != 0) {
        UMD_THROW(error::RuntimeError, "DMA size must be a multiple of 4.");
    }

    if (!bar2) {
        UMD_THROW(error::RuntimeError, "BAR2 is not mapped.");
    }

    std::visit([&](auto& strategy) { strategy.h2d_transfer(bar2, dma_buffer, dst, src, size); }, dma_strategy_);
}

}  // namespace tt::umd
