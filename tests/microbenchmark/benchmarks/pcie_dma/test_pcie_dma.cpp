// SPDX-FileCopyrightText: © 2025 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include <fmt/base.h>
#include <gtest/gtest.h>
#include <nanobench.h>
#include <sys/mman.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include "common/microbenchmark_utils.hpp"
#include "umd/device/chip/chip.hpp"
#include "umd/device/chip_helpers/sysmem_buffer.hpp"
#include "umd/device/chip_helpers/sysmem_manager.hpp"
#include "umd/device/cluster.hpp"
#include "umd/device/cluster_descriptor.hpp"
#include "umd/device/pcie/pci_device.hpp"
#include "umd/device/soc_descriptor.hpp"
#include "umd/device/types/arch.hpp"
#include "umd/device/types/cluster_descriptor_types.hpp"
#include "umd/device/types/cluster_types.hpp"
#include "umd/device/types/core_coordinates.hpp"

using namespace tt;
using namespace tt::umd;
using namespace tt::umd::test::utils;

constexpr ChipId CHIP_ID = 0;

TEST(MicrobenchmarkPCIeDMA, DRAM) {
    auto bench = ankerl::nanobench::Bench().title("DMA_DRAM").unit("byte");
    const uint64_t ADDRESS = 0x0;
    const std::vector<size_t> BATCH_SIZES = {
        4,
        8,
        16,
        32,
        1 * ONE_KIB,
        2 * ONE_KIB,
        4 * ONE_KIB,
        8 * ONE_KIB,
        16 * ONE_KIB,
        32 * ONE_KIB,
        1 * ONE_MIB,
        2 * ONE_MIB,
        4 * ONE_MIB,
        8 * ONE_MIB,
        16 * ONE_MIB,
        32 * ONE_MIB,
        1 * ONE_GIB,
    };
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord dram_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::DRAM)[0];
    for (size_t batch_size : BATCH_SIZES) {
        std::vector<uint8_t> pattern(batch_size);
        bench.batch(batch_size).name(fmt::format("DMA, write, {} bytes", batch_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, dram_core, ADDRESS);
        });
    }
    if (d2h_supported) {
        for (size_t batch_size : BATCH_SIZES) {
            std::vector<uint8_t> readback(batch_size);
            bench.batch(batch_size).name(fmt::format("DMA, read, {} bytes", batch_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, dram_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

TEST(MicrobenchmarkPCIeDMA, Tensix) {
    auto bench = ankerl::nanobench::Bench().title("DMA_Tensix").unit("byte");
    const uint64_t ADDRESS = 0x0;
    const std::vector<size_t> BATCH_SIZES = {4, 8, 1 * ONE_KIB, 2 * ONE_KIB, 4 * ONE_KIB, 8 * ONE_KIB, 1 * ONE_MIB};
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord tensix_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::TENSIX)[0];
    for (size_t batch_size : BATCH_SIZES) {
        std::vector<uint8_t> pattern(batch_size);
        bench.batch(batch_size).name(fmt::format("DMA, write, {} bytes", batch_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, tensix_core, ADDRESS);
        });
    }
    if (d2h_supported) {
        for (size_t batch_size : BATCH_SIZES) {
            std::vector<uint8_t> readback(batch_size);
            bench.batch(batch_size).name(fmt::format("DMA, read, {} bytes", batch_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, tensix_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

TEST(MicrobenchmarkPCIeDMA, Ethernet) {
    auto bench = ankerl::nanobench::Bench().title("DMA_Ethernet").unit("byte");
    const uint64_t ADDRESS = 0x20000;  // 128 KiB
    const std::vector<size_t> BATCH_SIZES = {4, 8, 1 * ONE_KIB, 2 * ONE_KIB, 4 * ONE_KIB, 8 * ONE_KIB, 128 * ONE_KIB};
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord eth_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::ETH)[0];
    for (size_t batch_size : BATCH_SIZES) {
        std::vector<uint8_t> pattern(batch_size);
        bench.batch(batch_size).name(fmt::format("DMA, write, {} bytes", batch_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, eth_core, ADDRESS);
        });
    }
    if (d2h_supported) {
        for (size_t batch_size : BATCH_SIZES) {
            std::vector<uint8_t> readback(batch_size);
            bench.batch(batch_size).name(fmt::format("DMA, read, {} bytes", batch_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, eth_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

TEST(MicrobenchmarkPCIeDMA, DRAMSweepSizes) {
    auto bench = ankerl::nanobench::Bench().title("DMA_DRAM_Sweep").unit("byte");
    const uint64_t ADDRESS = 0x0;
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord dram_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::DRAM)[0];
    const uint64_t LIMIT_BUF_SIZE = ONE_GIB;
    for (uint64_t buf_size = 4; buf_size <= LIMIT_BUF_SIZE; buf_size *= 2) {
        std::vector<uint8_t> pattern(buf_size);
        bench.batch(buf_size).name(fmt::format("DMA, write, {} bytes", buf_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, dram_core, ADDRESS);
        });
        if (d2h_supported) {
            std::vector<uint8_t> readback(buf_size);
            bench.batch(buf_size).name(fmt::format("DMA, read, {} bytes", buf_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, dram_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

TEST(MicrobenchmarkPCIeDMA, TensixSweepSizes) {
    auto bench = ankerl::nanobench::Bench().title("DMA_Tensix_Sweep").unit("byte");
    const uint64_t ADDRESS = 0x0;
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord tensix_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::TENSIX)[0];
    const uint64_t LIMIT_BUF_SIZE = ONE_MIB;
    for (uint64_t buf_size = 4; buf_size <= LIMIT_BUF_SIZE; buf_size *= 2) {
        std::vector<uint8_t> pattern(buf_size);
        bench.batch(buf_size).name(fmt::format("DMA, write, {} bytes", buf_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, tensix_core, ADDRESS);
        });
        if (d2h_supported) {
            std::vector<uint8_t> readback(buf_size);
            bench.batch(buf_size).name(fmt::format("DMA, read, {} bytes", buf_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, tensix_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

TEST(MicrobenchmarkPCIeDMA, EthernetSweepSizes) {
    auto bench = ankerl::nanobench::Bench().title("DMA_Ethernet_Sweep").unit("byte");
    const uint64_t ADDRESS = 0x20000;  // 128 KiB
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord eth_core = cluster->get_soc_descriptor(CHIP_ID).get_cores(CoreType::ETH)[0];
    const uint64_t LIMIT_BUF_SIZE = 128 * ONE_KIB;
    for (uint64_t buf_size = 4; buf_size <= LIMIT_BUF_SIZE; buf_size *= 2) {
        std::vector<uint8_t> pattern(buf_size);
        bench.batch(buf_size).name(fmt::format("DMA, write, {} bytes", buf_size)).run([&]() {
            cluster->dma_write_to_device(pattern.data(), pattern.size(), CHIP_ID, eth_core, ADDRESS);
        });
        if (d2h_supported) {
            std::vector<uint8_t> readback(buf_size);
            bench.batch(buf_size).name(fmt::format("DMA, read, {} bytes", buf_size)).run([&]() {
                cluster->dma_read_from_device(readback.data(), readback.size(), CHIP_ID, eth_core, ADDRESS);
            });
        }
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// READ-ONLY (2026-07-07): per-channel layout of the BH DMA controller register file (0x100-stride blocks).
TEST(MicrobenchmarkPCIeDMA, Bar2DmaChannelLayout) {
    std::vector<int> ids = PCIDevice::enumerate_devices();
    ASSERT_FALSE(ids.empty());
    PCIDevice dev(ids.at(0));
    volatile uint8_t* bar2 = reinterpret_cast<volatile uint8_t*>(dev.bar2_uc);
    ASSERT_NE(bar2, nullptr);
    auto rd = [&](uint32_t off) -> uint32_t { return *reinterpret_cast<volatile uint32_t*>(bar2 + off); };
    const uint32_t LIMIT = dev.bar2_uc_size < 0x2000 ? static_cast<uint32_t>(dev.bar2_uc_size) : 0x2000;
    for (uint32_t b = 0; b + 0x100 <= LIMIT; b += 0x100) {
        fmt::print(
            ">>> ch@{:#06x}: EN={:#x} +3c={:#010x} +80={:#x} +88={:#010x} SAR={:#010x} DAR={:#010x} XFER={:#x}\n",
            b, rd(b + 0x00), rd(b + 0x3c), rd(b + 0x80), rd(b + 0x88), rd(b + 0x20), rd(b + 0x28), rd(b + 0x1c));
    }
    // Fine-grained dump of write channel 0 (0x000) and read channel 0 (0x100), every 4 bytes, to expose
    // control/mode/linked-list-pointer registers.
    for (uint32_t chbase : {0x000u, 0x100u}) {
        fmt::print(">>> --- full register block ch@{:#06x} ---\n", chbase);
        for (uint32_t o = 0; o < 0x100; o += 4) {
            uint32_t v = rd(chbase + o);
            if (v != 0 && v != 0xffffffff) fmt::print(">>>   +{:#05x} = {:#010x}\n", o, v);
        }
    }
}

// PROOF/SWEEP (2026-07-07): device->host DMA on Blackhole. Prefill device DRAM (NoC write), then try the D2H
// transfer on each 0x100-stride channel block (env TT_D2H_CH) to find which writes host memory. Failure is
// contained (zeros into our own buffer, no hang) as demonstrated on ch@0x100.
TEST(MicrobenchmarkPCIeDMA, D2HDmaProof) {
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const ChipId chip = *cluster->get_target_mmio_device_ids().begin();
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord dram_core = cluster->get_soc_descriptor(chip).get_cores(CoreType::DRAM)[0];
    const uint64_t ADDR = 0x0;
    const size_t N = 64;

    std::vector<uint8_t> pattern(N);
    for (size_t i = 0; i < N; i++) pattern[i] = static_cast<uint8_t>(0xC0 + i);
    cluster->write_to_device(pattern.data(), pattern.size(), chip, dram_core, ADDR);
    std::vector<uint8_t> via_dma(N, 0xEE);
    cluster->dma_read_from_device(via_dma.data(), via_dma.size(), chip, dram_core, ADDR);
    fmt::print(">>> small D2H (64B) bit-exact match={}\n", via_dma == pattern);

    // At scale: prefill 512 KiB (one DMA staging chunk) via the fast H2D DMA, read it back via D2H, verify
    // EVERY byte, time it. Set TT_D2H_CHANNELS=N to split the read across N concurrent write channels.
    const size_t BIG = 512 * ONE_KIB;
    std::vector<uint8_t> src(BIG);
    for (size_t i = 0; i < BIG; i++) src[i] = static_cast<uint8_t>((i * 131u + 7u) & 0xff);
    cluster->dma_write_to_device(src.data(), src.size(), chip, dram_core, ADDR);
    std::vector<uint8_t> dst(BIG, 0);
    auto t0 = std::chrono::steady_clock::now();
    cluster->dma_read_from_device(dst.data(), dst.size(), chip, dram_core, ADDR);
    auto t1 = std::chrono::steady_clock::now();
    double us = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000.0;
    bool full_match = (dst == src);
    size_t mismatches = 0;
    for (size_t i = 0; i < BIG; i++) mismatches += (dst[i] != src[i]);
    fmt::print(">>> 1MiB D2H: full_match={} mismatched_bytes={} time={:.1f}us throughput={:.2f} GB/s\n",
               full_match, mismatches, us, (double)BIG / (us * 1e3));
    fmt::print(full_match ? ">>> *** D2H DMA WORKS ON BLACKHOLE (bit-exact, 1 MiB) ***\n"
                          : ">>> D2H 1MiB MISMATCH\n");
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// MEMCPY-FREE (2026-07-07): zero-copy D2H on Blackhole. DMA device DRAM straight into a hugepage-backed
// SysmemBuffer (no IOMMU needed) that the app reads directly — no staging buffer, no memcpy. Compares against
// the copy-path dma_read_from_device to show the memcpy elimination.
TEST(MicrobenchmarkPCIeDMA, D2HZeroCopy) {
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const ChipId chip = *cluster->get_target_mmio_device_ids().begin();
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord dram_core = cluster->get_soc_descriptor(chip).get_cores(CoreType::DRAM)[0];
    const CoreCoord dram_tr =
        cluster->get_soc_descriptor(chip).get_cores(CoreType::DRAM, CoordSystem::TRANSLATED)[0];
    const uint64_t ADDR = 0x0;
    const size_t N = 8 * ONE_MIB;
    const int K = 20;

    // Prefill device DRAM with a verifiable pattern (fast H2D DMA).
    std::vector<uint8_t> src(N);
    for (size_t i = 0; i < N; i++) src[i] = static_cast<uint8_t>((i * 131u + 7u) & 0xff);
    cluster->dma_write_to_device(src.data(), src.size(), chip, dram_core, ADDR);

    std::vector<uint8_t> copy_dst(N, 0);
    try {
        SysmemManager* sm = cluster->get_chip(chip)->get_sysmem_manager();
        std::unique_ptr<SysmemBuffer> sb = sm->allocate_sysmem_buffer(1ULL << 30, /*map_to_noc=*/false);
        uint8_t* host = static_cast<uint8_t*>(sb->get_buffer_va());
        std::memset(host, 0, N);

        // Correctness (one shot each).
        cluster->dma_read_from_device(copy_dst.data(), N, chip, dram_core, ADDR);
        sb->dma_read_from_device(0, N, dram_tr, ADDR);
        fmt::print(">>> correctness: copy-path match={} zero-copy match={}\n",
                   copy_dst == src, std::memcmp(host, src.data(), N) == 0);

        // Timed (warm + averaged over K iters).
        cluster->dma_read_from_device(copy_dst.data(), N, chip, dram_core, ADDR);  // warm
        auto c0 = std::chrono::steady_clock::now();
        for (int k = 0; k < K; k++) cluster->dma_read_from_device(copy_dst.data(), N, chip, dram_core, ADDR);
        auto c1 = std::chrono::steady_clock::now();
        double copy_us = std::chrono::duration_cast<std::chrono::nanoseconds>(c1 - c0).count() / 1000.0 / K;

        sb->dma_read_from_device(0, N, dram_tr, ADDR);  // warm
        auto z0 = std::chrono::steady_clock::now();
        for (int k = 0; k < K; k++) sb->dma_read_from_device(0, N, dram_tr, ADDR);
        auto z1 = std::chrono::steady_clock::now();
        double zc_us = std::chrono::duration_cast<std::chrono::nanoseconds>(z1 - z0).count() / 1000.0 / K;

        fmt::print(">>> copy-path  D2H ({}MiB): {:.1f}us = {:.2f} GB/s (DMA + memcpy)\n",
                   N / (1024 * 1024), copy_us, static_cast<double>(N) / (copy_us * 1e3));
        fmt::print(">>> zero-copy  D2H ({}MiB): {:.1f}us = {:.2f} GB/s (memcpy-free)\n",
                   N / (1024 * 1024), zc_us, static_cast<double>(N) / (zc_us * 1e3));
        fmt::print(">>> *** memcpy-free speedup: {:.2f}x ***\n", copy_us / zc_us);
    } catch (const std::exception& e) {
        fmt::print(">>> zero-copy setup failed: {}\n", e.what());
    }
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// This test measures bandwidth of IO using PCIe DMA engine where user buffer is mapped through IOMMU
// and no copying is done. It uses SysmemManager to map the buffer and then uses DMA to transfer data
// to and from the device.
TEST(MicrobenchmarkPCIeDMA, DRAMZeroCopy) {
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    if (!PCIDevice(pci_device_ids.at(0)).is_iommu_enabled()) {
        GTEST_SKIP() << "Skipping test since IOMMU is not enabled on the system.";
    }

    auto bench = ankerl::nanobench::Bench().title("DMA_DRAM_ZeroCopy").unit("byte");
    const uint64_t ADDRESS = 0x0;
    const size_t BUFFER_SIZE = ONE_MIB;
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>(ClusterOptions{
        .num_host_mem_ch_per_mmio_device = 0,
    });
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const ChipId mmio_chip = *cluster->get_target_mmio_device_ids().begin();
    SysmemManager* sysmem_manager = cluster->get_chip(mmio_chip)->get_sysmem_manager();
    std::unique_ptr<SysmemBuffer> sysmem_buffer = sysmem_manager->allocate_sysmem_buffer(200 * ONE_MIB);
    const CoreCoord dram_core = cluster->get_soc_descriptor(mmio_chip).get_cores(CoreType::DRAM)[0];

    bench.batch(BUFFER_SIZE).name(fmt::format("DMA, write, {} bytes", BUFFER_SIZE)).run([&]() {
        sysmem_buffer->dma_write_to_device(0, BUFFER_SIZE, dram_core, ADDRESS);
    });
    if (d2h_supported) {
        bench.batch(BUFFER_SIZE).name(fmt::format("DMA, read, {} bytes", BUFFER_SIZE)).run([&]() {
            sysmem_buffer->dma_read_from_device(0, BUFFER_SIZE, dram_core, ADDRESS);
        });
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// Test the PCIe DMA controller by using it to write random fixed-size pattern
// to address 0 of Tensix core, then reading them back and verifying.
// This test measures bandwidth of IO using PCIe DMA engine without overhead of copying data into DMA buffer.
TEST(MicrobenchmarkPCIeDMA, TensixZeroCopy) {
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    if (!PCIDevice(pci_device_ids.at(0)).is_iommu_enabled()) {
        GTEST_SKIP() << "Skipping test since IOMMU is not enabled on the system.";
    }

    auto bench = ankerl::nanobench::Bench().title("DMA_Tensix_ZeroCopy").unit("byte").epochs(1).epochIterations(1000);
    const uint64_t ADDRESS = 0x0;
    const size_t BUFFER_SIZE = ONE_MIB;
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>(ClusterOptions{
        .num_host_mem_ch_per_mmio_device = 0,
    });
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);

    const ChipId mmio_chip = *cluster->get_target_mmio_device_ids().begin();
    SysmemManager* sysmem_manager = cluster->get_chip(mmio_chip)->get_sysmem_manager();
    std::unique_ptr<SysmemBuffer> sysmem_buffer = sysmem_manager->allocate_sysmem_buffer(2 * ONE_MIB);
    const CoreCoord tensix_core = cluster->get_soc_descriptor(mmio_chip).get_cores(CoreType::TENSIX)[0];

    bench.batch(BUFFER_SIZE).name(fmt::format("DMA, write, {} bytes", BUFFER_SIZE)).run([&]() {
        sysmem_buffer->dma_write_to_device(0, BUFFER_SIZE, tensix_core, ADDRESS);
    });
    if (d2h_supported) {
        bench.batch(BUFFER_SIZE).name(fmt::format("DMA, read, {} bytes", BUFFER_SIZE)).run([&]() {
            sysmem_buffer->dma_read_from_device(0, BUFFER_SIZE, tensix_core, ADDRESS);
        });
    }
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// This test measures bandwidth of IO using PCIe DMA engine where user buffer is mapped through IOMMU
// and no copying is done. It uses SysmemManager to map the buffer and then uses DMA to transfer data
// to and from the device.
TEST(MicrobenchmarkPCIeDMA, TensixMapBufferZeroCopy) {
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    if (!PCIDevice(pci_device_ids.at(0)).is_iommu_enabled()) {
        GTEST_SKIP() << "Skipping test since IOMMU is not enabled on the system.";
    }

    auto bench = ankerl::nanobench::Bench().title("DMA_Tensix_MapBuffer_ZeroCopy").unit("byte");
    const uint64_t ADDRESS = 0x0;
    const size_t BUFFER_SIZE = ONE_MIB;
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>(ClusterOptions{
        .num_host_mem_ch_per_mmio_device = 0,
    });
    const bool d2h_supported = true;
    cluster->set_power_state(DevicePowerState::BUSY);
    const ChipId mmio_chip = *cluster->get_target_mmio_device_ids().begin();
    SysmemManager* sysmem_manager = cluster->get_chip(mmio_chip)->get_sysmem_manager();
    void* mapping =
        mmap(nullptr, BUFFER_SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_PRIVATE | MAP_POPULATE, -1, 0);
    const CoreCoord tensix_core = cluster->get_soc_descriptor(mmio_chip).get_cores(CoreType::TENSIX)[0];

    bench.batch(BUFFER_SIZE).name(fmt::format("DMA, write, {} bytes", BUFFER_SIZE)).run([&]() {
        std::unique_ptr<SysmemBuffer> sysmem_buffer = sysmem_manager->map_sysmem_buffer(mapping, BUFFER_SIZE);
        sysmem_buffer->dma_write_to_device(0, BUFFER_SIZE, tensix_core, ADDRESS);
    });
    if (d2h_supported) {
        bench.batch(BUFFER_SIZE).name(fmt::format("DMA, read, {} bytes", BUFFER_SIZE)).run([&]() {
            std::unique_ptr<SysmemBuffer> sysmem_buffer = sysmem_manager->map_sysmem_buffer(mapping, BUFFER_SIZE);
            sysmem_buffer->dma_read_from_device(0, BUFFER_SIZE, tensix_core, ADDRESS);
        });
    }
    munmap(mapping, BUFFER_SIZE);
    test::utils::export_results(bench);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}

// MULTI-CHANNEL (2026-07-07): test concurrent multi-channel D2H DMA bandwidth.
// This test is designed to benchmark the aggregate bandwidth when TT_D2H_CHANNELS is implemented
// in the UMD readback path. It pushes a 32 MiB read which should automatically trigger internal chunking
// and round-robin across all available write channels.
TEST(MicrobenchmarkPCIeDMA, D2HMultiChannelBandwidth) {
    std::unique_ptr<Cluster> cluster = std::make_unique<Cluster>();
    const ChipId chip = *cluster->get_target_mmio_device_ids().begin();
    cluster->set_power_state(DevicePowerState::BUSY);
    const CoreCoord dram_core = cluster->get_soc_descriptor(chip).get_cores(CoreType::DRAM)[0];
    const uint64_t ADDR = 0x0;
    
    // 32 MiB is large enough to saturate PCIe Gen4 and force deep chunking across all 8 channels.
    const size_t N = 32 * ONE_MIB;
    
    std::vector<uint8_t> src(N);
    for (size_t i = 0; i < N; i++) src[i] = static_cast<uint8_t>((i * 131u + 7u) & 0xff);
    
    // Warmup + Prefill
    cluster->dma_write_to_device(src.data(), src.size(), chip, dram_core, ADDR);
    
    std::vector<uint8_t> dst(N, 0);
    
    // Timed read
    auto t0 = std::chrono::steady_clock::now();
    cluster->dma_read_from_device(dst.data(), dst.size(), chip, dram_core, ADDR);
    auto t1 = std::chrono::steady_clock::now();
    
    double us = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000.0;
    bool full_match = (dst == src);
    
    fmt::print(">>> {}MiB D2H Multi-Channel (aggregate): match={} time={:.1f}us throughput={:.2f} GB/s\n",
               N / (1024 * 1024), full_match, us, (double)N / (us * 1e3));
               
    ASSERT_TRUE(full_match);
    cluster->set_power_state(DevicePowerState::LONG_IDLE);
}
