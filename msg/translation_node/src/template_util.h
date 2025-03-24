/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <memory>
#include <utility>
#include <type_traits>
#include <vector>

/**
 * Helper struct to store template parameter packs
 */
template <typename... Args>
struct Pack {
};

/**
 * Struct for a template parameter pack with access to the individual types
 */
template<typename ...Types>
struct TypesArray {
	template<typename T, typename...OtherTypes>
	struct TypeHelper {
		using Type = T;
		using Next = TypeHelper<OtherTypes..., void>;
	};

	using Type1 = typename TypeHelper<Types...>::Type;
	using Type2 = typename TypeHelper<Types...>::Next::Type;
	using Type3 = typename TypeHelper<Types...>::Next::Next::Type;
	using Type4 = typename TypeHelper<Types...>::Next::Next::Next::Type;
	using Type5 = typename TypeHelper<Types...>::Next::Next::Next::Next::Type;
	using Type6 = typename TypeHelper<Types...>::Next::Next::Next::Next::Next::Type;

	using args = Pack<Types...>;
};


/**
 * Helper for call_translation_function()
 */
template<typename F, typename MessageType, typename... ArgsIn, typename... ArgsOut, size_t... Is, size_t... Os>
inline void call_translation_function_impl(F f, Pack<ArgsIn...>, Pack<ArgsOut...>,
        const std::vector<std::shared_ptr<MessageType>>& messages_in,
		std::vector<std::shared_ptr<MessageType>>& messages_out,
		std::integer_sequence<size_t, Is...>, std::integer_sequence<size_t, Os...>)
{
	f(*static_cast<const ArgsIn*>(messages_in[Is].get())..., *static_cast<ArgsOut*>(messages_out[Os].get())...);
}

/**
 * Call a translation function F which takes the arguments (const ArgsIn&..., ArgsOut&...),
 * by passing messages_in and messages_out as arguments.
 * Note that sizeof(ArgsIn) == messages_in.length() && sizeof(ArgsOut) == messages_out.length() must hold.
 */
template<typename F, typename MessageType, typename... ArgsIn, typename... ArgsOut>
inline void call_translation_function(F f, Pack<ArgsIn...> pack_in, Pack<ArgsOut...> pack_out,
									  const std::vector<std::shared_ptr<MessageType>>& messages_in,
									  std::vector<std::shared_ptr<MessageType>>& messages_out) {
	call_translation_function_impl(f, pack_in, pack_out, messages_in, messages_out,
								   std::index_sequence_for<ArgsIn...>{}, std::index_sequence_for<ArgsOut...>{});
}
