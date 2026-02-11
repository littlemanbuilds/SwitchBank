/**
 * MIT License
 *
 * @brief Small compatibility helpers for SwitchBank to support Arduino cores with limited libstdc++.
 *
 * @file SwitchBank_Compatibility.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-01-30
 * @copyright Copyright © 2026 Little Man Builds
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#if defined(__has_include)
#if __has_include(<type_traits>)
#include <type_traits>
#define SWITCHBANK_HAS_TYPE_TRAITS 1
#else
#define SWITCHBANK_HAS_TYPE_TRAITS 0
#endif
#else
#define SWITCHBANK_HAS_TYPE_TRAITS 0
#endif

namespace switchbank
{
    namespace compat
    {
        /**
         * @brief Minimal enable_if fallback used by SwitchBank API constraints.
         */
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
    } ///< namespace compat
} ///< namespace switchbank
