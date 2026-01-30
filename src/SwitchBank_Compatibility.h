/**
 * MIT License
 *
 * @brief Small compatibility helpers for SwitchBank to support Arduino cores with limited libstdc++.
 *
 * @file SwitchBank_Compat.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-01-30
 * @copyright Copyright © 2026 Little Man Builds
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#if __has_include(<type_traits>)
#include <type_traits>
#define SWITCHBANK_HAS_TYPE_TRAITS 1
#else
#define SWITCHBANK_HAS_TYPE_TRAITS 0
#endif

namespace switchbank::compat
{
#if SWITCHBANK_HAS_TYPE_TRAITS
    template <bool B, typename T = void>
    using enable_if = std::enable_if<B, T>;
#else
    template <bool B, typename T = void>
    struct enable_if
    {
    };
    template <typename T>
    struct enable_if<true, T>
    {
        using type = T;
    };
#endif

    template <bool B, typename T = void>
    using enable_if_t = typename enable_if<B, T>::type;

#if SWITCHBANK_HAS_TYPE_TRAITS
    template <typename T>
    struct is_enum
    {
        static constexpr bool value = std::is_enum<T>::value;
    };
#else
    // Best-effort: if no <type_traits>, we can't do robust enum detection.
    template <typename T>
    struct is_enum
    {
        static constexpr bool value = false;
    };
#endif
} ///< namespace switchbank::compat
