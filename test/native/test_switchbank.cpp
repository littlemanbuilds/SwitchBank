/**
 * MIT License
 *
 * @brief Native deterministic contract tests for SwitchBank.
 *
 * @file test_switchbank.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright © 2026 Little Man Builds
 */

#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "SwitchBank.h"
#include "SwitchBank_Factory.h"

static unsigned g_tests = 0u;
static unsigned g_assertions = 0u;
#define CHECK(x) do { ++g_assertions; if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (0)
#define TEST(name) do { ++g_tests; std::cout << "[test] " << name << "\n"; } while (0)

struct PerKeyCtx
{
    std::uint32_t levels{0u};
    bool fail{false};
    unsigned reads{0u};
};

static bool read_bool(void *ctx, std::uint8_t key)
{
    PerKeyCtx *c = static_cast<PerKeyCtx *>(ctx);
    ++c->reads;
    return ((c->levels >> key) & 1u) != 0u;
}

static SwitchBankReadResult read_result(void *ctx, std::uint8_t key)
{
    PerKeyCtx *c = static_cast<PerKeyCtx *>(ctx);
    ++c->reads;
    if (c->fail)
    {
        return SwitchBankReadResult::failure();
    }
    return SwitchBankReadResult::success(((c->levels >> key) & 1u) != 0u);
}

struct PackedCtx
{
    std::uint32_t levels{0u};
    bool fail{false};
    unsigned reads{0u};
};

static SwitchBankPackedReadResult read_packed(void *ctx)
{
    PackedCtx *c = static_cast<PackedCtx *>(ctx);
    ++c->reads;
    return c->fail ? SwitchBankPackedReadResult::failure()
                   : SwitchBankPackedReadResult::success(c->levels);
}

int main()
{
    const std::uint8_t keys3[3] = {0u, 1u, 2u};

    TEST("missing reader fails closed instead of synthesizing active-low ON");
    {
        SwitchBank<3> bank(keys3, 0u, 0x7u, static_cast<SwitchBank<3>::ReadPinFn>(nullptr), nullptr);
        CHECK(!bank.configured());
        CHECK(!bank.sync(10u));
        CHECK(bank.peekValue() == 0u);
        CHECK(!bank.valid());
        CHECK(!bank.hasSample());
        CHECK(bank.lastError() == SwitchBankReadError::MissingReader);
        CHECK(!bank.status().configured);
        CHECK(bank.sequence() == 0u);
    }

    TEST("active-low and active-high polarity are normalized correctly");
    {
        PerKeyCtx c;
        c.levels = 5u;
        SwitchBank<3> low(keys3, 0u, 0x7u, &read_bool, &c, nullptr);
        CHECK(low.configured());
        CHECK(low.sync(1u));
        CHECK(low.peekValue() == 2u);
        SwitchBank<3> high(keys3, 0u, 0x0u, &read_bool, &c, nullptr);
        CHECK(high.sync(1u));
        CHECK(high.peekValue() == 5u);
    }

    TEST("validity-aware reader failure preserves stable and debounce state");
    {
        PerKeyCtx c;
        c.levels = 7u; // active-low => 000
        SwitchBank<3> bank(keys3, 10u, 0x7u, &read_result, &c, nullptr);
        CHECK(bank.sync(100u));
        CHECK(bank.peekValue() == 0u);
        CHECK(bank.sequence() == 1u);
        c.levels = 6u; // bit 0 wants ON
        CHECK(!bank.update(105u));
        CHECK(bank.sequence() == 2u);
        c.fail = true;
        CHECK(!bank.update(120u));
        CHECK(!bank.valid());
        CHECK(bank.peekValue() == 0u);
        CHECK(bank.sequence() == 2u);
        CHECK(bank.lastError() == SwitchBankReadError::AcquisitionFailed);
        c.fail = false;
        CHECK(!bank.update(121u)); // Recovery starts a fresh observed debounce window.
        CHECK(bank.peekValue() == 0u);
        CHECK(bank.sequence() == 3u);
        CHECK(!bank.update(130u));
        CHECK(bank.update(131u));
        CHECK(bank.peekValue() == 1u);
        CHECK(bank.sequence() == 5u);
    }

    TEST("sample freshness advances on healthy stable reads while change time does not");
    {
        PerKeyCtx c;
        c.levels = 7u;
        SwitchBank<3> bank(keys3, 0u, 0x7u, &read_result, &c, nullptr);
        CHECK(bank.sync(10u));
        CHECK(bank.sampleMs() == 10u);
        CHECK(bank.changeMs() == 0u);
        CHECK(!bank.update(20u));
        CHECK(bank.sampleMs() == 20u);
        CHECK(bank.sequence() == 2u);
        CHECK(bank.changeSequence() == 0u);
        CHECK(bank.changeMs() == 0u);
    }

    TEST("failed acquisition records attempt and error time without changing sample time");
    {
        PerKeyCtx c;
        c.levels = 7u;
        SwitchBank<3> bank(keys3, 0u, 0x7u, &read_result, &c, nullptr);
        CHECK(bank.sync(10u));
        c.fail = true;
        CHECK(!bank.update(25u));
        const SwitchBankStatus st = bank.status();
        CHECK(!st.valid);
        CHECK(st.sample_ms == 10u);
        CHECK(st.attempt_ms == 25u);
        CHECK(st.error_ms == 25u);
        CHECK(st.sequence == 1u);
    }

    TEST("coherent packed reader performs exactly one acquisition per update");
    {
        PackedCtx c;
        c.levels = 7u;
        auto bank = makeSwitchBankPacked<3>(keys3, 0u, &read_packed, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(c.reads == 1u);
        c.levels = 2u;
        CHECK(bank.update(2u));
        CHECK(c.reads == 2u);
        CHECK(bank.peekValue() == 5u);
    }

    TEST("packed acquisition failure cannot synthesize a partial selector combination");
    {
        PackedCtx c;
        c.levels = 15u;
        const std::uint8_t keys4[4] = {0u, 1u, 2u, 3u};
        auto bank = makeSwitchBankPacked<4>(keys4, 0u, &read_packed, &c, nullptr);
        CHECK(bank.sync(1u));
        c.fail = true;
        c.levels = 10u;
        CHECK(!bank.update(2u));
        CHECK(bank.peekValue() == 0u);
        c.fail = false;
        CHECK(bank.update(3u));
        CHECK(bank.peekValue() == 5u);
    }

    TEST("packed selector transitions do not expose sequential impossible intermediates");
    {
        // Physical HIGH mask 1010 -> logical active-low 0101, then 1100 -> 0011.
        // These represent two complete contact states acquired atomically.
        PackedCtx c;
        const std::uint8_t keys4[4] = {0u, 1u, 2u, 3u};
        c.levels = 10u;
        auto bank = makeSwitchBankPacked<4>(keys4, 0u, &read_packed, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(bank.peekValue() == 5u);
        c.levels = 12u;
        CHECK(bank.update(2u));
        CHECK(bank.peekValue() == 3u);
        CHECK(c.reads == 2u);
    }

    TEST("per-bit debounce rejects bounce and commits after stable window");
    {
        PerKeyCtx c;
        c.levels = 1u; // active-low single input OFF
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 10u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(100u));
        c.levels = 0u;
        CHECK(!bank.update(101u));
        c.levels = 1u;
        CHECK(!bank.update(105u));
        c.levels = 0u;
        CHECK(!bank.update(108u));
        CHECK(!bank.update(117u));
        CHECK(bank.update(118u));
        CHECK(bank.peekValue() == 1u);
    }

    TEST("debounce and polling arithmetic survive uint32 millis wrap");
    {
        PerKeyCtx c;
        const std::uint8_t key[1] = {0u};
        c.levels = 1u;
        SwitchBank<1> bank(key, 5u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(0xFFFFFFFCu));
        bank.setMinPollMs(2u);
        c.levels = 0u;
        CHECK(!bank.update(0xFFFFFFFEu));
        CHECK(!bank.update(0u));
        CHECK(bank.update(3u));
        CHECK(bank.peekValue() == 1u);
    }

    TEST("sync increments generation without resetting acquisition or change sequences");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(bank.generation() == 1u);
        c.levels = 0u;
        CHECK(bank.update(2u));
        CHECK(bank.sequence() == 2u);
        CHECK(bank.changeSequence() == 1u);
        CHECK(bank.sync(3u));
        CHECK(bank.generation() == 2u);
        CHECK(bank.sequence() == 3u);
        CHECK(bank.changeSequence() == 1u);
        CHECK(bank.changedMask() == 0u);
    }

    TEST("explicit edge consumption has no hidden read side effect");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(1u));
        c.levels = 0u;
        CHECK(bank.update(2u));
        CHECK(bank.peekValue() == 1u);
        CHECK(bank.changed());
        CHECK(bank.peekValue() == 1u);
        CHECK(bank.changed());
        const SwitchBankEdges e = bank.consumeEdges();
        CHECK(e.changed == 1u);
        CHECK(e.rising == 1u);
        CHECK(e.falling == 0u);
        CHECK(e.change_sequence == 1u);
        CHECK(!bank.changed());
        CHECK(bank.changedMask() == 0u);
    }

    TEST("change sequence reveals multiple transitions even if edges are sampled late");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(1u));
        c.levels = 0u;
        CHECK(bank.update(2u));
        c.levels = 1u;
        CHECK(bank.update(3u));
        CHECK(bank.changeSequence() == 2u);
        CHECK(bank.peekValue() == 0u);
    }

    TEST("out-of-range bit helpers fail closed");
    {
        PerKeyCtx c;
        c.levels = 0u;
        SwitchBank<3> bank(keys3, 0u, 0u, &read_bool, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(!bank.isOn(3u));
        CHECK(!bank.rose(99u));
        CHECK(!bank.fell(255u));
    }

    TEST("reverse-order packing is deterministic");
    {
        PerKeyCtx c;
        c.levels = 1u;
        SwitchBank<3, -1, true> bank(keys3, 0u, 0u, &read_bool, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(bank.peekValue() == 4u);
    }

    TEST("runtime polarity change is edge-free and rollback-safe on read failure");
    {
        PerKeyCtx c;
        c.levels = 7u;
        SwitchBank<3> bank(keys3, 0u, 0x7u, &read_result, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(bank.peekValue() == 0u);
        CHECK(bank.setActiveLowMask(0u, 2u));
        CHECK(bank.peekValue() == 7u);
        CHECK(bank.changedMask() == 0u);
        const std::uint32_t mask_before = bank.activeLowMask();
        c.fail = true;
        CHECK(!bank.setActiveLowMask(0x7u, 3u));
        CHECK(bank.activeLowMask() == mask_before);
        CHECK(bank.peekValue() == 7u);
    }

    TEST("commissioning force commit is explicit and reports failed acquisition");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 100u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(1u));
        c.levels = 0u;
        CHECK(bank.forceCommitForCommissioning(2u));
        CHECK(bank.peekValue() == 1u);
        c.fail = true;
        CHECK(!bank.forceCommitForCommissioning(3u));
        CHECK(bank.peekValue() == 1u);
    }

    TEST("snapshot separates sample, change, sequence and generation metadata");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(10u));
        c.levels = 0u;
        CHECK(bank.update(20u));
        const SwitchBank<1>::SwitchBankSnapshot s = bank.snapshot();
        CHECK(s.value == 1u);
        CHECK(s.sample_ms == 20u);
        CHECK(s.change_ms == 20u);
        CHECK(s.t_ms == 20u);
        CHECK(s.seq == 2u);
        CHECK(s.change_seq == 1u);
        CHECK(s.generation == 1u);
        CHECK(s.valid);
    }

    TEST("min poll throttling does not create fake acquisitions");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        CHECK(bank.sync(100u));
        bank.setMinPollMs(10u);
        CHECK(!bank.update(105u));
        CHECK(bank.sequence() == 1u);
        CHECK(c.reads == 1u);
        CHECK(!bank.update(110u));
        CHECK(bank.sequence() == 2u);
        CHECK(c.reads == 2u);
    }

    TEST("first update is not throttled before any acquisition timestamp exists");
    {
        PerKeyCtx c;
        c.levels = 1u;
        const std::uint8_t key[1] = {0u};
        SwitchBank<1> bank(key, 0u, 0x1u, &read_result, &c, nullptr);
        bank.setMinPollMs(10u);
        CHECK(!bank.update(1u));
        CHECK(bank.valid());
        CHECK(bank.sequence() == 1u);
        CHECK(c.reads == 1u);
        CHECK(!bank.update(5u));
        CHECK(bank.sequence() == 1u);
        CHECK(c.reads == 1u);
    }

    TEST("packed 32-bit bank masks safely without shift overflow");
    {
        std::uint8_t keys32[32]{};
        for (std::uint8_t i = 0u; i < 32u; ++i) keys32[i] = i;
        PackedCtx c;
        c.levels = 0x80000001u;
        SwitchBank<32> bank(keys32, 0u, 0u, &read_packed, &c, nullptr);
        CHECK(bank.sync(1u));
        CHECK(bank.peekValue() == 0x80000001u);
    }

    TEST("all 4-bit electrical/polarity combinations map exactly");
    {
        const std::uint8_t keys4[4] = {0u, 1u, 2u, 3u};
        for (std::uint32_t electrical = 0u; electrical < 16u; ++electrical)
        {
            for (std::uint32_t mask = 0u; mask < 16u; ++mask)
            {
                PackedCtx c;
                c.levels = electrical;
                SwitchBank<4> bank(keys4, 0u, mask, &read_packed, &c, nullptr);
                CHECK(bank.sync(1u));
                const std::uint32_t expected = ((~electrical & mask) | (electrical & ~mask)) & 0x0Fu;
                CHECK(bank.peekValue() == expected);
            }
        }
    }

    TEST("reverse-order mapping obeys output-coordinate polarity for all 4-bit patterns");
    {
        const std::uint8_t keys4[4] = {0u, 1u, 2u, 3u};
        for (std::uint32_t electrical = 0u; electrical < 16u; ++electrical)
        {
            for (std::uint32_t mask = 0u; mask < 16u; ++mask)
            {
                PackedCtx c;
                c.levels = electrical;
                SwitchBank<4, -1, true> bank(keys4, 0u, mask, &read_packed, &c, nullptr);
                CHECK(bank.sync(1u));
                std::uint32_t expected = 0u;
                for (std::uint8_t i = 0u; i < 4u; ++i)
                {
                    const std::uint8_t bi = static_cast<std::uint8_t>(3u - i);
                    const bool high = ((electrical >> i) & 1u) != 0u;
                    const bool active_low = ((mask >> bi) & 1u) != 0u;
                    if (active_low ? !high : high) expected |= (1u << bi);
                }
                CHECK(bank.peekValue() == expected);
            }
        }
    }

    TEST("builder supports validity-aware and packed reader paths");
    {
        PerKeyCtx per;
        per.levels = 7u;
        SwitchBankBuilder<3> result_builder(keys3);
        auto result_bank = result_builder.withDebounce(5u).withResultReader(&read_result, &per).build();
        CHECK(result_bank.sync(1u));
        CHECK(result_bank.valid());

        PackedCtx packed;
        packed.levels = 7u;
        SwitchBankBuilder<3> packed_builder(keys3);
        auto packed_bank = packed_builder.withPackedReader(&read_packed, &packed).build();
        CHECK(packed_bank.sync(1u));
        CHECK(packed.reads == 1u);
    }

    TEST("PW_PVT DIP and gear constructor shapes remain source-compatible");
    {
        PerKeyCtx c;
        const std::uint8_t dip_keys[3] = {0u, 1u, 2u};
        const std::uint8_t gear_keys[4] = {3u, 4u, 5u, 6u};
        SwitchBank<3> dip(dip_keys, 20u, 0x7u, &read_bool, &c, nullptr);
        SwitchBank<4> gear(gear_keys, 20u, 0xFu, &read_bool, &c, nullptr);
        CHECK(dip.sync(1u));
        CHECK(gear.sync(1u));
        CHECK(dip.valid());
        CHECK(gear.valid());
    }

    TEST("gear-selector allowed combinations are preserved as complete packed states");
    {
        // Consumer-level legal masks for R/FWD/D1/D3 contacts in PW_PVT.
        const std::uint32_t legal[] = {0u, 1u, 2u, 6u, 10u};
        PackedCtx c;
        const std::uint8_t keys4[4] = {0u, 1u, 2u, 3u};
        auto bank = makeSwitchBankPackedMasked<4>(keys4, 0u, &read_packed, &c, 0u, nullptr);
        for (std::size_t i = 0u; i < (sizeof(legal) / sizeof(legal[0])); ++i)
        {
            c.levels = legal[i];
            if (i == 0u) CHECK(bank.sync(static_cast<std::uint32_t>(i)));
            else (void)bank.update(static_cast<std::uint32_t>(i));
            CHECK(bank.peekValue() == legal[i]);
        }
    }

    std::cout << "PASS: " << g_tests << " tests / " << g_assertions << " assertions\n";
    return 0;
}
