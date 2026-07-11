/*
 * SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "umd/device/tt_device/protocol/pcie_dma/blackhole_dma_transfer.hpp"

#include <fmt/base.h>

#include <chrono>
#include <cstdlib>
#include <string>

#include "umd/device/pcie/pci_device.hpp"
#include "umd/device/utils/error.hpp"

namespace tt::umd {

void BlackholeDmaTransfer::d2h_transfer(
    volatile uint8_t* bar2, DmaBuffer& dma_buffer, uint64_t dst, uint32_t src, size_t size) {
    // device->host DMA on Blackhole. The BH controller exposes 16 channel blocks (BAR2 stride 0x100) as 8
    // interleaved read/write pairs: EVEN blocks (0x000,0x200,...) are write channels (device->host), ODD
    // blocks (0x100,0x300,...) are read channels (host->device, used by h2d). D2H is the h2d register
    // sequence with source/dest swapped — SAR = device AXI (32-bit), DAR = host physical (64-bit) — on a
    // write channel. Use write channel 0 (0x000); h2d uses read channel 0 (0x100), so the two never collide.
    // Verified bit-exact on Blackhole p150a, 2026-07-07.
    uint32_t base = 0x000;
    if (const char* e = std::getenv("TT_D2H_CH")) {
        base = static_cast<uint32_t>(std::strtoul(e, nullptr, 0));
    }
    const uint32_t EN_OFF = base + 0x00;
    const uint32_t DOORBELL_OFF = base + 0x04;
    const uint32_t XFERSIZE_OFF = base + 0x1C;
    const uint32_t SAR_LOW_OFF = base + 0x20;
    const uint32_t SAR_HIGH_OFF = base + 0x24;
    const uint32_t DAR_LOW_OFF = base + 0x28;
    const uint32_t DAR_HIGH_OFF = base + 0x2C;
    const uint32_t INT_SETUP_OFF = base + 0x88;
    const uint32_t MSI_STOP_LOW_OFF = base + 0x90;
    const uint32_t MSI_STOP_HIGH_OFF = base + 0x94;
    const uint32_t MSI_ABORT_LOW_OFF = base + 0xA0;
    const uint32_t MSI_ABORT_HIGH_OFF = base + 0xA4;
    const uint32_t MSI_MSGD_OFF = base + 0xA8;
    static constexpr uint32_t DMA_TIMEOUT_MS = 10000;

    auto write_reg = [&](uint32_t offset, uint32_t value) {
        *reinterpret_cast<volatile uint32_t*>(bar2 + offset) = value;
    };
    auto read_reg = [&](uint32_t offset) -> uint32_t { return *reinterpret_cast<volatile uint32_t*>(bar2 + offset); };

    write_reg(INT_SETUP_OFF, 0x28);
    write_reg(MSI_STOP_LOW_OFF, static_cast<uint32_t>(dma_buffer.completion_pa & 0xFFFFFFFF));
    write_reg(MSI_STOP_HIGH_OFF, static_cast<uint32_t>((dma_buffer.completion_pa >> 32) & 0xFFFFFFFF));
    write_reg(MSI_ABORT_LOW_OFF, static_cast<uint32_t>((dma_buffer.completion_pa + sizeof(uint32_t)) & 0xFFFFFFFF));
    write_reg(
        MSI_ABORT_HIGH_OFF,
        static_cast<uint32_t>(((dma_buffer.completion_pa + sizeof(uint32_t)) >> 32) & 0xFFFFFFFF));
    // Ordered completion: have the engine write a magic value to the host completion flag when done. Because
    // that write is issued AFTER the data writes on the same path, PCIe ordering guarantees all D2H data has
    // landed in host DRAM once the CPU observes the magic (polling the device XFERSIZE register does NOT).
    static constexpr uint32_t DMA_COMPLETION_VALUE = 0xfaca;
    const bool wait_xfersize = std::getenv("TT_D2H_WAIT_XFER") != nullptr;  // A/B: old (unordered) path
    volatile uint32_t* completion = reinterpret_cast<volatile uint32_t*>(dma_buffer.completion);
    *completion = 0;
    write_reg(MSI_MSGD_OFF, DMA_COMPLETION_VALUE);
    write_reg(EN_OFF, 0x1);
    // Source = device AXI address (BH uses a 32-bit device address space).
    write_reg(SAR_LOW_OFF, src);
    write_reg(SAR_HIGH_OFF, 0);
    // Destination = host physical address (64-bit).
    write_reg(DAR_LOW_OFF, static_cast<uint32_t>(dst & 0xFFFFFFFF));
    write_reg(DAR_HIGH_OFF, static_cast<uint32_t>((dst >> 32) & 0xFFFFFFFF));
    // Set transfer size and ring the doorbell to start the DMA.
    write_reg(XFERSIZE_OFF, static_cast<uint32_t>(size));
    write_reg(DOORBELL_OFF, 0x1);

    auto start = std::chrono::steady_clock::now();
    for (;;) {
        bool done = wait_xfersize ? (read_reg(XFERSIZE_OFF) == 0) : (*completion == DMA_COMPLETION_VALUE);
        if (done) {
            break;
        }
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed_ms > DMA_TIMEOUT_MS) {
            UMD_THROW(error::RuntimeError, "D2H DMA timeout.");
        }
    }
    if (std::getenv("TT_D2H_DEBUG")) {
        fmt::print(
            stderr, "[d2h] completion=0x{:x} xfersize=0x{:x}\n", *completion, read_reg(XFERSIZE_OFF));
    }
}

void BlackholeDmaTransfer::h2d_transfer(
    volatile uint8_t* bar2, DmaBuffer& dma_buffer, uint32_t dst, uint64_t src, size_t size) {
    static constexpr uint32_t EN_OFF_RDCH_0 = 0x100;
    static constexpr uint32_t DOORBELL_OFF_RDCH_0 = 0x104;
    static constexpr uint32_t XFERSIZE_OFF_RDCH_0 = 0x11C;
    static constexpr uint32_t SAR_LOW_OFF_RDCH_0 = 0x120;
    static constexpr uint32_t SAR_HIGH_OFF_RDCH_0 = 0x124;
    static constexpr uint32_t DAR_LOW_OFF_RDCH_0 = 0x128;
    static constexpr uint32_t DAR_HIGH_OFF_RDCH_0 = 0x12C;
    static constexpr uint32_t INT_SETUP_OFF_RDCH_0 = 0x188;
    static constexpr uint32_t MSI_STOP_LOW_OFF_RDCH_0 = 0x190;
    static constexpr uint32_t MSI_STOP_HIGH_OFF_RDCH_0 = 0x194;
    static constexpr uint32_t MSI_ABORT_LOW_OFF_RDCH_0 = 0x1A0;
    static constexpr uint32_t MSI_ABORT_HIGH_OFF_RDCH_0 = 0x1A4;
    static constexpr uint32_t MSI_MSGD_OFF_RDCH_0 = 0x1A8;
    static constexpr uint32_t DMA_TIMEOUT_MS = 10000;

    auto write_reg = [&](uint32_t offset, uint32_t value) {
        *reinterpret_cast<volatile uint32_t*>(bar2 + offset) = value;
    };

    auto read_reg = [&](uint32_t offset) -> uint32_t { return *reinterpret_cast<volatile uint32_t*>(bar2 + offset); };

    // Configure interrupt setup: enable local interrupt (bit 3) and remote stop interrupt (bit 5).
    write_reg(INT_SETUP_OFF_RDCH_0, 0x28);
    // Set the MSI write address for the DMA "done" interrupt to the completion flag physical address.
    write_reg(MSI_STOP_LOW_OFF_RDCH_0, static_cast<uint32_t>(dma_buffer.completion_pa & 0xFFFFFFFF));
    write_reg(MSI_STOP_HIGH_OFF_RDCH_0, static_cast<uint32_t>((dma_buffer.completion_pa >> 32) & 0xFFFFFFFF));
    // Set the MSI write address for the DMA "abort" interrupt to the word after the completion flag.
    write_reg(
        MSI_ABORT_LOW_OFF_RDCH_0, static_cast<uint32_t>((dma_buffer.completion_pa + sizeof(uint32_t)) & 0xFFFFFFFF));
    write_reg(
        MSI_ABORT_HIGH_OFF_RDCH_0,
        static_cast<uint32_t>(((dma_buffer.completion_pa + sizeof(uint32_t)) >> 32) & 0xFFFFFFFF));
    // MSI message data written on completion (unused for polling, set to 0).
    write_reg(MSI_MSGD_OFF_RDCH_0, 0);
    // Enable the DMA read channel.
    write_reg(EN_OFF_RDCH_0, 0x1);
    // Set the source address (host physical address of the DMA buffer).
    write_reg(SAR_LOW_OFF_RDCH_0, static_cast<uint32_t>(src & 0xFFFFFFFF));
    write_reg(SAR_HIGH_OFF_RDCH_0, static_cast<uint32_t>((src >> 32) & 0xFFFFFFFF));
    // Set the destination address (device AXI address). BH uses a 32-bit device address space.
    write_reg(DAR_LOW_OFF_RDCH_0, dst);
    write_reg(DAR_HIGH_OFF_RDCH_0, 0);
    // Set transfer size and ring the doorbell to start the DMA.
    write_reg(XFERSIZE_OFF_RDCH_0, static_cast<uint32_t>(size));
    write_reg(DOORBELL_OFF_RDCH_0, 0x1);

    // WARNING: Busy-wait poll. Consider adding _mm_pause() or adaptive polling to reduce
    // CPU and memory bus contention.
    auto start = std::chrono::steady_clock::now();
    while (read_reg(XFERSIZE_OFF_RDCH_0) != 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

        if (elapsed_ms > DMA_TIMEOUT_MS) {
            UMD_THROW(error::RuntimeError, "DMA timeout.");
        }
    }
}

}  // namespace tt::umd
