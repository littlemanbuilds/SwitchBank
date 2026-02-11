/**
 * MIT License
 *
 * @brief Factories and mask helpers for SwitchBank.
 *
 * @file SwitchBank_Factory.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-01
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <SwitchBank.h>

#if defined(__has_include)
#if __has_include(<initializer_list>)
#include <initializer_list>
#define SWITCHBANK_HAS_INITIALIZER_LIST 1
#endif
#endif

#ifndef SWITCHBANK_HAS_INITIALIZER_LIST
#define SWITCHBANK_HAS_INITIALIZER_LIST 0
#endif

// ---- Mask helpers ---- //

/**
 * @brief Build an active-low mask where all bits are active-low for N inputs.
 * @tparam N Number of inputs (1..32).
 * @return Active-low mask (bit=1 means active-low).
 */
template <size_t N>
constexpr uint32_t mask_all_active_low() noexcept
{
    static_assert(N > 0 && N <= 32, "mask_all_active_low<N>: N must be 1..32");
    return (N == 32) ? 0xFFFFFFFFu : ((1u << N) - 1u);
}

/**
 * @brief Build an active-low mask where all bits are active-high for N inputs.
 * @tparam N Number of inputs (1..32).
 * @return Active-low mask (all zeros => active-high).
 */
template <size_t N>
constexpr uint32_t mask_all_active_high() noexcept
{
    static_assert(N > 0 && N <= 32, "mask_all_active_high<N>: N must be 1..32");
    return 0u;
}

/**
 * @brief Build a mask where the listed indices are active-high; all others active-low.
 * @tparam N Total inputs (1..32).
 * @tparam K Number of indices provided.
 * @param active_high_indices Indices (0..N-1) that should be active-high.
 * @return Active-low mask (bit=1 means active-low).
 */
template <size_t N, size_t K>
#if __cplusplus >= 201402L
constexpr uint32_t mask_from_active_high_indices(const uint8_t (&active_high_indices)[K]) noexcept
#else
inline uint32_t mask_from_active_high_indices(const uint8_t (&active_high_indices)[K]) noexcept
#endif
{
    static_assert(N > 0 && N <= 32, "mask_from_active_high_indices<N>: N must be 1..32");
    uint32_t mask = mask_all_active_low<N>();
    for (size_t i = 0; i < K; ++i)
    {
        const uint8_t idx = active_high_indices[i];
        if (idx < N)
        {
            mask &= ~(1u << idx);
        } ///< Clear bit → active-high.
    }
    return mask;
}

/**
 * @brief Runtime variant (initializer_list) of mask_from_active_high_indices().
 * @tparam N Total inputs (1..32).
 * @param active_high_indices Indices (0..N-1) that should be active-high.
 * @return Active-low mask (bit=1 means active-low).
 */
#if SWITCHBANK_HAS_INITIALIZER_LIST
template <size_t N>
inline uint32_t mask_from_active_high_indices(std::initializer_list<uint8_t> active_high_indices) noexcept
{
    uint32_t mask = mask_all_active_low<N>();
    for (uint8_t idx : active_high_indices)
    {
        if (idx < N)
        {
            mask &= ~(1u << idx);
        } ///< Clear bit → active-high.
    }
    return mask;
}
#endif

/**
 * @brief Runtime variant without STL containers.
 * @tparam N Total inputs (1..32).
 * @return Active-low mask with no active-high indices (all active-low).
 */
template <size_t N>
inline uint32_t mask_from_active_high_indices() noexcept
{
    return mask_all_active_low<N>();
}

/**
 * @brief Runtime variant without STL containers.
 * @tparam N Total inputs (1..32).
 * @param active_high_index First active-high index (0..N-1).
 * @param rest Additional active-high indices (0..N-1).
 * @return Active-low mask (bit=1 means active-low).
 */
template <size_t N, typename... Rest>
inline uint32_t mask_from_active_high_indices(uint8_t active_high_index, Rest... rest) noexcept
{
    uint32_t mask = mask_from_active_high_indices<N>(rest...);
    if (active_high_index < N)
    {
        mask &= ~(1u << active_high_index);
    } ///< Clear bit -> active-high.
    return mask;
}

/**
 * @brief Build a mask from a per-bit boolean array.
 * @tparam N Total inputs (1..32).
 * @param active_low Per-bit polarity array (N entries).
 * @return Active-low mask (bit=1 means active-low).
 */
template <size_t N>
#if __cplusplus >= 201402L
constexpr uint32_t mask_from_active_low_array(const bool (&active_low)[N]) noexcept
#else
inline uint32_t mask_from_active_low_array(const bool (&active_low)[N]) noexcept
#endif
{
    static_assert(N > 0 && N <= 32, "mask_from_active_low_array<N>: N must be 1..32");
    uint32_t mask = 0u;
    for (size_t i = 0; i < N; ++i)
    {
        if (active_low[i])
        {
            mask |= (1u << i);
        } ///< Set bit → active-low.
    }
    return mask;
}

// ---- Factories (runtime polarity) ---- //

/**
 * @brief Create a SwitchBank with a fast per-key reader (no context). LSB-first packing.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_pin Per-key electrical reader (key → level HIGH/LOW).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N> makeSwitchBankPins(const uint8_t (&keys)[N],
                                        uint16_t debounce_ms,
                                        typename SwitchBank<N>::ReadPinFn read_pin,
                                        SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N>(keys, debounce_ms, mask_all_active_low<N>(), read_pin, time_fn);
}

/**
 * @brief Create a SwitchBank with a context-aware reader. LSB-first packing.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_ctx Reader taking (ctx, key) → electrical level HIGH/LOW.
 * @param ctx Opaque context pointer (nullable).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N> makeSwitchBankCtx(const uint8_t (&keys)[N],
                                       uint16_t debounce_ms,
                                       typename SwitchBank<N>::ReadFn read_ctx,
                                       void *ctx = nullptr,
                                       SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N>(keys, debounce_ms, mask_all_active_low<N>(), read_ctx, ctx, time_fn);
}

/**
 * @brief Create a SwitchBank with an explicit runtime mask (already computed). LSB-first packing.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_pin Per-key electrical reader.
 * @param active_low_mask Active-low mask (bit=1 means active-low).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N> makeSwitchBankPinsMasked(const uint8_t (&keys)[N],
                                              uint16_t debounce_ms,
                                              typename SwitchBank<N>::ReadPinFn read_pin,
                                              uint32_t active_low_mask,
                                              SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N>(keys, debounce_ms, active_low_mask, read_pin, time_fn);
}

/**
 * @brief Create a SwitchBank with an explicit runtime mask and a context-aware reader. LSB-first packing.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_ctx Context-aware reader (ctx, key) → electrical level HIGH/LOW.
 * @param ctx Opaque context pointer (nullable).
 * @param active_low_mask Active-low mask (bit=1 means active-low).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N> makeSwitchBankCtxMasked(const uint8_t (&keys)[N],
                                             uint16_t debounce_ms,
                                             typename SwitchBank<N>::ReadFn read_ctx,
                                             void *ctx,
                                             uint32_t active_low_mask,
                                             SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N>(keys, debounce_ms, active_low_mask, read_ctx, ctx, time_fn);
}

// ---- Factories (runtime polarity, reversed bit order) ---- //

/**
 * @brief Create a SwitchBank with a fast per-key reader and **reversed** bit order.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_pin Per-key electrical reader (key → level HIGH/LOW).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N, -1, true> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N, -1, true> makeSwitchBankPinsRev(const uint8_t (&keys)[N],
                                                     uint16_t debounce_ms,
                                                     typename SwitchBank<N, -1, true>::ReadPinFn read_pin,
                                                     SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N, -1, true>(keys, debounce_ms, mask_all_active_low<N>(), read_pin, time_fn);
}

/**
 * @brief Create a SwitchBank with a context-aware reader and **reversed** bit order.
 * @tparam N Number of inputs (1..32).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_ctx Context-aware reader (ctx, key) → level HIGH/LOW.
 * @param ctx Opaque context pointer (nullable).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N, -1, true> instance (by value, zero-heap).
 */
template <size_t N>
inline SwitchBank<N, -1, true> makeSwitchBankCtxRev(const uint8_t (&keys)[N],
                                                    uint16_t debounce_ms,
                                                    typename SwitchBank<N, -1, true>::ReadFn read_ctx,
                                                    void *ctx = nullptr,
                                                    SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N, -1, true>(keys, debounce_ms, mask_all_active_low<N>(), read_ctx, ctx, time_fn);
}

// ---- Factories (compile-time polarity with optional reversed order) ---- //

/**
 * @brief Create a SwitchBank with **compile-time** polarity and optional reversed bit order.
 * @tparam N Number of inputs (1..32).
 * @tparam PolMask Compile-time active-low mask (bit=1 means active-low).
 * @tparam ReverseOrder If true, reverse bit packing (bit 0 ← keys[N-1]).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_pin Per-key electrical reader (key → level HIGH/LOW).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N, PolMask, ReverseOrder> instance (by value, zero-heap).
 */
template <size_t N, int64_t PolMask, bool ReverseOrder = false>
inline SwitchBank<N, PolMask, ReverseOrder>
makeSwitchBankPinsCT(const uint8_t (&keys)[N],
                     uint16_t debounce_ms,
                     typename SwitchBank<N, PolMask, ReverseOrder>::ReadPinFn read_pin,
                     SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N, PolMask, ReverseOrder>(keys,
                                                debounce_ms,
                                                /*runtime mask ignored*/ 0u,
                                                read_pin,
                                                time_fn);
}

/**
 * @brief Create a SwitchBank with **compile-time** polarity and optional reversed bit order (context-aware reader).
 * @tparam N Number of inputs (1..32).
 * @tparam PolMask Compile-time active-low mask (bit=1 means active-low).
 * @tparam ReverseOrder If true, reverse bit packing (bit 0 ← keys[N-1]).
 * @param keys Key array (pins/identifiers).
 * @param debounce_ms Debounce window in ms (0 disables).
 * @param read_ctx Context-aware reader (ctx, key) → level HIGH/LOW.
 * @param ctx Opaque context pointer (nullable).
 * @param time_fn Optional time source for no-arg update()/sync()/commit().
 * @return SwitchBank<N, PolMask, ReverseOrder> instance (by value, zero-heap).
 */
template <size_t N, int64_t PolMask, bool ReverseOrder = false>
inline SwitchBank<N, PolMask, ReverseOrder>
makeSwitchBankCtxCT(const uint8_t (&keys)[N],
                    uint16_t debounce_ms,
                    typename SwitchBank<N, PolMask, ReverseOrder>::ReadFn read_ctx,
                    void *ctx = nullptr,
                    SwitchBankHandler::TimeFn time_fn = nullptr)
{
    return SwitchBank<N, PolMask, ReverseOrder>(keys,
                                                debounce_ms,
                                                /*runtime mask ignored*/ 0u,
                                                read_ctx,
                                                ctx,
                                                time_fn);
}

// ---- Lightweight builder (runtime polarity) ---- //

/**
 * @brief Fluent builder for SwitchBank creation (runtime polarity).
 * @tparam N Number of inputs (1..32).
 */
template <size_t N>
struct SwitchBankBuilder
{
    const uint8_t (&keys)[N];                   ///< Keys to use (pins/identifiers).
    uint16_t debounce_ms{0};                    ///< Debounce in ms (0 disables).
    uint32_t mask{mask_all_active_low<N>()};    ///< Active-low mask (bit=1 means active-low).
    SwitchBankHandler::TimeFn time_fn{nullptr}; ///< Optional time source for no-arg update()/sync()/commit().

    typename SwitchBank<N>::ReadPinFn read_pin{nullptr}; ///< Per-key reader (fast path).
    typename SwitchBank<N>::ReadFn read_ctx{nullptr};    ///< Context-aware reader.
    void *ctx{nullptr};                                  ///< Reader context (nullable).

    /**
     * @brief Construct a builder bound to a key array.
     * @param key_array Keys to use (pins/identifiers).
     */
    explicit SwitchBankBuilder(const uint8_t (&key_array)[N]) noexcept
        : keys(key_array) {}

    /**
     * @brief Set debounce window in ms.
     * @param ms Debounce time (0 disables).
     * @return *this for chaining.
     */
    SwitchBankBuilder &withDebounce(uint16_t ms)
    {
        debounce_ms = ms;
        return *this;
    }

    /**
     * @brief Set all inputs to active-low.
     * @return *this for chaining.
     */
    SwitchBankBuilder &withAllActiveLow()
    {
        mask = mask_all_active_low<N>();
        return *this;
    }

    /**
     * @brief Set all inputs to active-high.
     * @return *this for chaining.
     */
    SwitchBankBuilder &withAllActiveHigh()
    {
        mask = mask_all_active_high<N>();
        return *this;
    }

    /**
     * @brief Provide an explicit active-low mask (bit=1 means active-low).
     * @param m Mask to use.
     * @return *this for chaining.
     */
    SwitchBankBuilder &withActiveLowMask(uint32_t m)
    {
        mask = m;
        return *this;
    }

    /**
     * @brief Compute mask from a list of indices that are active-high; others active-low.
     * @tparam K Number of indices provided.
     * @param idx Indices (0..N-1) that are active-high.
     * @return *this for chaining.
     */
    template <size_t K>
    SwitchBankBuilder &withActiveHighIndices(const uint8_t (&idx)[K])
    {
        mask = mask_from_active_high_indices<N, K>(idx);
        return *this;
    }

    /**
     * @brief Inject a time source for no-arg update()/sync()/commit().
     * @param tf Time function returning milliseconds.
     * @return *this for chaining.
     */
    SwitchBankBuilder &withTime(SwitchBankHandler::TimeFn tf)
    {
        time_fn = tf;
        return *this;
    }

    /**
     * @brief Use a per-key reader (fast path).
     * @param rp Reader function: key → electrical level HIGH/LOW.
     * @return *this for chaining.
     */
    SwitchBankBuilder &withReader(typename SwitchBank<N>::ReadPinFn rp)
    {
        read_pin = rp;
        read_ctx = nullptr;
        ctx = nullptr;
        return *this;
    }

    /**
     * @brief Use a context-aware reader.
     * @param rc Reader function: (ctx, key) → electrical level HIGH/LOW.
     * @param c Context pointer (nullable).
     * @return *this for chaining.
     */
    SwitchBankBuilder &withReader(typename SwitchBank<N>::ReadFn rc, void *c = nullptr)
    {
        read_ctx = rc;
        ctx = c;
        read_pin = nullptr;
        return *this;
    }

    /**
     * @brief Check whether a reader has been configured.
     * @return true if either a per-key or context-aware reader is set.
     */
    bool hasReader() const noexcept
    {
        return (read_pin != nullptr) || (read_ctx != nullptr);
    }

    /**
     * @brief Build the SwitchBank instance (runtime polarity, LSB-first).
     *        Requires withReader(...) to be configured first.
     * @return SwitchBank<N> instance (by value, zero-heap).
     */

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    using OnCommitFn =
        void (*)(const typename SwitchBank<N>::SwitchBankSnapshot &) noexcept;
    /// Optional: callback fired when a new stable value commits.
    SwitchBankBuilder &onCommit(OnCommitFn cb) noexcept
    {
        on_commit_ = cb;
        return *this;
    }
#endif
    SwitchBank<N> build() const
    {
        // Require an explicit reader to avoid silently creating an inert bank.
        assert(hasReader());

        if (read_pin)
        {
            SwitchBank<N> b(keys, debounce_ms, mask, read_pin, time_fn);
#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
            if (on_commit_)
                b.setOnCommit(on_commit_);
#endif
            return b;
        }
        SwitchBank<N> b(keys, debounce_ms, mask, read_ctx, ctx, time_fn);
#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
        if (on_commit_)
            b.setOnCommit(on_commit_);
#endif
        return b;
    }

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    typename SwitchBank<N>::OnCommitFn on_commit_{nullptr};
#endif
};
