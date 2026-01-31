/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>
#include <src/graph.h>


TEST(graph, basic)
{
	struct NodeData {
		bool iterated{false};
		bool translated{false};
	};
	Graph<NodeData> graph;

	const int32_t message1_value = 3;
	const int32_t offset = 4;

	// Add 2 nodes
	const MessageIdentifier id1{"topic_name", 1};
	auto buffer1 = std::make_shared<int32_t>();
	*buffer1 = message1_value;
	EXPECT_TRUE(graph.addNodeIfNotExists(id1, {}, buffer1));
	EXPECT_FALSE(graph.addNodeIfNotExists(id1, {}, std::make_shared<int32_t>()));
	const MessageIdentifier id2{"topic_name", 4};
	auto buffer2 = std::make_shared<int64_t>();
	*buffer2 = 773;
	EXPECT_TRUE(graph.addNodeIfNotExists(id2, {}, buffer2));

	// Search nodes
	EXPECT_TRUE(graph.findNode(id1).has_value());
	EXPECT_TRUE(graph.findNode(id2).has_value());

	// Add 1 translation
	auto translation_cb = [&offset](const std::vector<MessageBuffer>& a, std::vector<MessageBuffer>& b) {
		auto a_value = static_cast<const int32_t*>(a[0].get());
		auto b_value = static_cast<int64_t*>(b[0].get());
		*b_value = *a_value + offset;
	};
	graph.addTranslation(translation_cb, {id1}, {id2});

	// Iteration from id1 must reach id2
	auto node1 = graph.findNode(id1).value();
	auto node2 = graph.findNode(id2).value();
	auto iterate_cb = [](const Graph<NodeData>::MessageNodePtr& node) {
		node->data().iterated = true;
	};
	graph.iterateBFS(node1, iterate_cb);
	EXPECT_TRUE(node1->data().iterated);
	EXPECT_TRUE(node2->data().iterated);
	node1->data().iterated = false;
	node2->data().iterated = false;

	// Iteration from id2 must not reach id1
	graph.iterateBFS(node2, iterate_cb);
	EXPECT_FALSE(node1->data().iterated);
	EXPECT_TRUE(node2->data().iterated);

	// Test translation
	graph.translate(node1,
					[](auto&& node) {
						assert(!node->data().translated);
						node->data().translated = true;
					});
	EXPECT_FALSE(node1->data().translated);
	EXPECT_TRUE(node2->data().translated);
	EXPECT_EQ(*buffer1, message1_value);
	EXPECT_EQ(*buffer2, message1_value + offset);
}


TEST(graph, multi_path)
{
	// Multiple paths with cycles
	struct NodeData {
		unsigned iterated_idx{0};
		bool translated{false};
	};
	Graph<NodeData> graph;

	static constexpr unsigned num_nodes = 6;
	std::array<MessageIdentifier, num_nodes> ids{{
			{"topic_name", 1},
			{"topic_name", 2},
			{"topic_name", 3},
			{"topic_name", 4},
			{"topic_name", 5},
			{"topic_name", 6},
	}};

	std::array<std::shared_ptr<int32_t>, num_nodes> buffers{{
			std::make_shared<int32_t>(),
			std::make_shared<int32_t>(),
			std::make_shared<int32_t>(),
			std::make_shared<int32_t>(),
			std::make_shared<int32_t>(),
			std::make_shared<int32_t>(),
	}};

	// Nodes
	for (unsigned i = 0; i < num_nodes; ++i) {
		EXPECT_TRUE(graph.addNodeIfNotExists(ids[i], {}, buffers[i]));
	}

	// Translations
	std::bitset<32> translated;

	auto get_translation_cb = [&translated](unsigned bit) {
		auto translation_cb = [&translated, bit](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
			auto a_value = static_cast<const int32_t *>(a[0].get());
			auto b_value = static_cast<int32_t *>(b[0].get());
			*b_value = *a_value | (1 << bit);
			translated.set(bit);
		};
		return translation_cb;
	};

	// Graph:
	// ___ 2 -- 3 -- 4
	// |        |
	// 1 _______|
	// |
	// 5
	// |
	// 6

	unsigned next_bit = 0;
	// Connect each node to the previous and next, except the last 3
	for (unsigned i=0; i < num_nodes - 3; ++i) {
		graph.addTranslation(get_translation_cb(next_bit++), {ids[i]}, {ids[i+1]});
		graph.addTranslation(get_translation_cb(next_bit++), {ids[i+1]}, {ids[i]});
	}

	// Connect the first to the 3rd as well
	graph.addTranslation(get_translation_cb(next_bit++), {ids[0]}, {ids[2]});
	graph.addTranslation(get_translation_cb(next_bit++), {ids[2]}, {ids[0]});

	// Connect the second last to the first one
	graph.addTranslation(get_translation_cb(next_bit++), {ids[0]}, {ids[num_nodes-2]});
	graph.addTranslation(get_translation_cb(next_bit++), {ids[num_nodes-2]}, {ids[0]});

	// Connect the second last to the last one
	graph.addTranslation(get_translation_cb(next_bit++), {ids[num_nodes-1]}, {ids[num_nodes-2]});
	graph.addTranslation(get_translation_cb(next_bit++), {ids[num_nodes-2]}, {ids[num_nodes-1]});

	unsigned iteration_idx = 1;
	graph.iterateBFS(graph.findNode(ids[0]).value(), [&iteration_idx](const Graph<NodeData>::MessageNodePtr& node) {
		assert(node->data().iterated_idx == 0);
		node->data().iterated_idx = iteration_idx++;
	});

	EXPECT_EQ(graph.findNode(ids[0]).value()->data().iterated_idx, 1);
	// We're a bit stricter than we would have to be: ids[1,2,4] would be allowed to have any of the values (2,3,4)
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().iterated_idx, 2);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().iterated_idx, 3);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().iterated_idx, 4);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().iterated_idx, 5);


	// Translation
	graph.translate(graph.findNode(ids[0]).value(),
					[](auto&& node) {
						assert(!node->data().translated);
						node->data().translated = true;
					});

	// All nodes should be translated except the first
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, false);
	for (unsigned i = 1; i < num_nodes; ++i) {
		EXPECT_EQ(graph.findNode(ids[i]).value()->data().translated, true) << "node[" << i << "]";
	}

	// Ensure the correct edges were used for translations
	EXPECT_EQ("00000000000000000000100101010001", translated.to_string());

	// Ensure correct translation path taken for each node (which is stored in the buffers),
	// and translation callback got called
	EXPECT_EQ(*buffers[0], 0);
	EXPECT_EQ(*buffers[1], 0b1);
	EXPECT_EQ(*buffers[2], 0b1000000);
	EXPECT_EQ(*buffers[3], 0b1010000);
	EXPECT_EQ(*buffers[4], 0b100000000);
	EXPECT_EQ(*buffers[5], 0b100100000000);

	for (unsigned i=0; i < num_nodes; ++i) {
		printf("node[%i]: translated: %i, buffer: %i\n", i, graph.findNode(ids[i]).value()->data().translated,
			   *buffers[i]);
	}
}

TEST(graph, multi_links) {
	// Multiple topics (merging / splitting)
	struct NodeData {
		bool translated{false};
	};
	Graph<NodeData> graph;

	static constexpr unsigned num_nodes = 6;
	std::array<MessageIdentifier, num_nodes> ids{{
			 {"topic1", 1},
			 {"topic2", 1},
			 {"topic1", 2},
			 {"topic3", 1},
			 {"topic4", 1},
			 {"topic1", 3},
	 }};

	std::array<std::shared_ptr<uint32_t>, num_nodes> buffers{{
			std::make_shared<uint32_t>(),
			std::make_shared<uint32_t>(),
			std::make_shared<uint32_t>(),
			std::make_shared<uint32_t>(),
			std::make_shared<uint32_t>(),
			std::make_shared<uint32_t>(),
	}};

	// Nodes
	for (unsigned i = 0; i < num_nodes; ++i) {
		EXPECT_TRUE(graph.addNodeIfNotExists(ids[i], {}, buffers[i]));
	}


	// Graph
	//       ___
	//   1 - | |       ---
	//       | | - 3 - | | - 6
	//   2 - | |       ---
	//   |   ---
	//   |   ___
	//   --- | | - 4
	//       | | - 5
	//       ---

	// Translations
	auto translation_cb_merge = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 2);
		assert(b.size() == 1);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto a_value2 = static_cast<const uint32_t *>(a[1].get());
		auto b_value = static_cast<uint32_t *>(b[0].get());
		*b_value = *a_value1 | *a_value2;
	};
	auto translation_cb_split = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 1);
		assert(b.size() == 2);
		auto a_value = static_cast<const uint32_t *>(a[0].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		auto b_value2 = static_cast<uint32_t *>(b[1].get());
		*b_value1 = *a_value & 0x0000ffffu;
		*b_value2 = *a_value & 0xffff0000u;
	};
	auto translation_cb_direct = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 1);
		assert(b.size() == 1);
		auto a_value = static_cast<const uint32_t *>(a[0].get());
		auto b_value = static_cast<uint32_t *>(b[0].get());
		*b_value = *a_value;
	};

	auto addTranslation = [&](const std::vector<MessageIdentifier>& inputs, const std::vector<MessageIdentifier>& outputs) {
		assert(inputs.size() <= 2);
		assert(outputs.size() <= 2);
		if (inputs.size() == 1) {
			if (outputs.size() == 1) {
				graph.addTranslation(translation_cb_direct, inputs, outputs);
				graph.addTranslation(translation_cb_direct, outputs, inputs);
			} else {
				graph.addTranslation(translation_cb_split, inputs, outputs);
				graph.addTranslation(translation_cb_merge, outputs, inputs);
			}
		} else {
			assert(outputs.size() == 1);
			graph.addTranslation(translation_cb_merge, inputs, outputs);
			graph.addTranslation(translation_cb_split, outputs, inputs);
		}
	};
	addTranslation({ids[0], ids[1]}, {ids[2]});
	addTranslation({ids[1]}, {ids[3], ids[4]});
	addTranslation({ids[2]}, {ids[5]});

	auto translate_node = [&](const MessageIdentifier& id) {
		graph.translate(graph.findNode(id).value(),
						[](auto&& node) {
							assert(!node->data().translated);
							node->data().translated = true;
						});

	};
	auto reset_translated = [&]() {
		for (const auto& id : ids) {
			graph.findNode(id).value()->data().translated = false;
		}
	};

	// Updating node 2 should trigger an output for nodes 4+5 (splitting)
	*buffers[0] = 0xa00000b0;
	*buffers[1] = 0x0f00000f;
	translate_node(ids[1]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, false);
	EXPECT_EQ(*buffers[3], 0x0000000f);
	EXPECT_EQ(*buffers[4], 0x0f000000);

	reset_translated();

	// Now updating node 1 should update nodes 3+6 (merging, both inputs available now)
	translate_node(ids[0]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, true);
	EXPECT_EQ(*buffers[2], 0xaf0000bf);
	EXPECT_EQ(*buffers[5], 0xaf0000bf);

	reset_translated();

	// Another update must not trigger any other updates
	translate_node(ids[0]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, false);

	reset_translated();

	// Backwards: updating node 6 should trigger updates for 1+2, but also 4+5
	*buffers[5] = 0xc00000d0;
	translate_node(ids[5]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, false);
	EXPECT_EQ(*buffers[0], 0x000000d0);
	EXPECT_EQ(*buffers[1], 0xc0000000);
	EXPECT_EQ(*buffers[2], 0xc00000d0);
	EXPECT_EQ(*buffers[3], 0);
	EXPECT_EQ(*buffers[4], 0xc0000000);
	EXPECT_EQ(*buffers[5], 0xc00000d0);
}

TEST(graph, multi_links2) {
	// Multiple topics (merging / splitting)
	struct NodeData {
		bool translated{false};
	};
	Graph<NodeData> graph;

	static constexpr unsigned num_nodes = 8;
	std::array<MessageIdentifier, num_nodes> ids{{
														 {"topic1", 1},
														 {"topic2", 1},
														 {"topic3", 1},
														 {"topic1", 2},
														 {"topic2", 2},
														 {"topic1", 3},
														 {"topic2", 3},
														 {"topic3", 3},
												 }};

	std::array<std::shared_ptr<uint32_t>, num_nodes> buffers{{
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
															 }};

	// Nodes
	for (unsigned i = 0; i < num_nodes; ++i) {
		EXPECT_TRUE(graph.addNodeIfNotExists(ids[i], {}, buffers[i]));
	}


	// Graph
	//       ___       ___
	//   1 - | |       | | - 6
	//       | | - 4 - | |
	//   2 - | |       | | - 7
	//       | | - 5 - | |
	//   3 - | |       | | - 8
	//       ---       ---

	// Translations
	auto translation_cb_32 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 3);
		assert(b.size() == 2);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto a_value2 = static_cast<const uint32_t *>(a[1].get());
		auto a_value3 = static_cast<const uint32_t *>(a[2].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		auto b_value2 = static_cast<uint32_t *>(b[1].get());
		*b_value1 = *a_value1 | *a_value2;
		*b_value2 = *a_value3;
	};
	auto translation_cb_23 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 2);
		assert(b.size() == 3);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto a_value2 = static_cast<const uint32_t *>(a[1].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		auto b_value2 = static_cast<uint32_t *>(b[1].get());
		auto b_value3 = static_cast<uint32_t *>(b[2].get());
		*b_value1 = *a_value1 & 0x0000ffffu;
		*b_value2 = *a_value1 & 0xffff0000u;
		*b_value3 = *a_value2;
	};
	graph.addTranslation(translation_cb_32, {ids[0], ids[1], ids[2]}, {ids[3], ids[4]});
	graph.addTranslation(translation_cb_23, {ids[3], ids[4]}, {ids[0], ids[1], ids[2]});

	graph.addTranslation(translation_cb_23, {ids[3], ids[4]}, {ids[5], ids[6], ids[7]});
	graph.addTranslation(translation_cb_32, {ids[5], ids[6], ids[7]}, {ids[3], ids[4]});


	auto translate_node = [&](const MessageIdentifier& id) {
		graph.translate(graph.findNode(id).value(),
						[](auto&& node) {
							assert(!node->data().translated);
							node->data().translated = true;
						});
	};
	auto reset_translated = [&]() {
		for (const auto& id : ids) {
			graph.findNode(id).value()->data().translated = false;
		}
	};

	// Updating nodes 1+2+3 should update nodes 6+7+8
	*buffers[0] = 0xa00000b0;
	*buffers[1] = 0x0f00000f;
	*buffers[2] = 0x0c00000c;
	translate_node(ids[1]);
	translate_node(ids[0]);
	translate_node(ids[2]);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[6]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[7]).value()->data().translated, true);
	EXPECT_EQ(*buffers[3], 0xa00000b0 | 0x0f00000f);
	EXPECT_EQ(*buffers[4], 0x0c00000c);
	EXPECT_EQ(*buffers[5], (0xa00000b0 | 0x0f00000f) & 0x0000ffffu);
	EXPECT_EQ(*buffers[6], (0xa00000b0 | 0x0f00000f) & 0xffff0000u);
	EXPECT_EQ(*buffers[7], 0x0c00000c);

	reset_translated();

	// Now updating nodes 6+7+8 should update nodes 1+2+3
	*buffers[5] = 0xa00000b0;
	*buffers[6] = 0x0f00000f;
	*buffers[7] = 0x0c00000c;
	translate_node(ids[5]);
	translate_node(ids[6]);
	translate_node(ids[7]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(*buffers[3], 0xa00000b0 | 0x0f00000f);
	EXPECT_EQ(*buffers[4], 0x0c00000c);
	EXPECT_EQ(*buffers[0], (0xa00000b0 | 0x0f00000f) & 0x0000ffffu);
	EXPECT_EQ(*buffers[1], (0xa00000b0 | 0x0f00000f) & 0xffff0000u);
	EXPECT_EQ(*buffers[2], 0x0c00000c);
}

TEST(graph, multi_links3) {
	// Multiple topics (cannot use the shortest path)
	struct NodeData {
		bool translated{false};
	};
	Graph<NodeData> graph;

	static constexpr unsigned num_nodes = 7;
	std::array<MessageIdentifier, num_nodes> ids{{
														 {"topic1", 1},
														 {"topic2", 1},
														 {"topic1", 2},
														 {"topic1", 3},
														 {"topic1", 4},
														 {"topic2", 4},
														 {"topic1", 5},
												 }};

	std::array<std::shared_ptr<uint32_t>, num_nodes> buffers{{
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
																	 std::make_shared<uint32_t>(),
															 }};

	// Nodes
	for (unsigned i = 0; i < num_nodes; ++i) {
		EXPECT_TRUE(graph.addNodeIfNotExists(ids[i], {}, buffers[i]));
	}


	// Graph
	//       ___       ___       ___       ___
	//   1 - | | - 3 - | | - 4 - | | - 5 - | | - 7
	//       | |       ---       ---       | |
	//       | |                           | |
	//   2 - | | --------------------- 6 - | |
	//       ---                           ---

	// Translations
	auto translation_cb_21 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 2);
		assert(b.size() == 1);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto a_value2 = static_cast<const uint32_t *>(a[1].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		*b_value1 = *a_value1 | *a_value2;
	};
	auto translation_cb_22 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 2);
		assert(b.size() == 2);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto a_value2 = static_cast<const uint32_t *>(a[1].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		auto b_value2 = static_cast<uint32_t *>(b[1].get());
		*b_value1 = *a_value1;
		*b_value2 = *a_value2;
	};
	auto translation_cb_12 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 1);
		assert(b.size() == 2);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		auto b_value2 = static_cast<uint32_t *>(b[1].get());
		*b_value1 = *a_value1 & 0x0000ffffu;
		*b_value2 = *a_value1 & 0xffff0000u;
	};
	auto translation_cb_11 = [](const std::vector<MessageBuffer> &a, std::vector<MessageBuffer> &b) {
		assert(a.size() == 1);
		assert(b.size() == 1);
		auto a_value1 = static_cast<const uint32_t *>(a[0].get());
		auto b_value1 = static_cast<uint32_t *>(b[0].get());
		*b_value1 = *a_value1 + 1;
	};
	graph.addTranslation(translation_cb_22, {ids[0], ids[1]}, {ids[2], ids[5]});
	graph.addTranslation(translation_cb_22, {ids[2], ids[5]}, {ids[0], ids[1]});
	graph.addTranslation(translation_cb_11, {ids[2]}, {ids[3]});
	graph.addTranslation(translation_cb_11, {ids[3]}, {ids[2]});
	graph.addTranslation(translation_cb_11, {ids[3]}, {ids[4]});
	graph.addTranslation(translation_cb_11, {ids[4]}, {ids[3]});
	graph.addTranslation(translation_cb_21, {ids[4], ids[5]}, {ids[6]});
	graph.addTranslation(translation_cb_12, {ids[6]}, {ids[4], ids[5]});


	auto translate_node = [&](const MessageIdentifier& id) {
		graph.translate(graph.findNode(id).value(),
						[](auto&& node) {
							assert(!node->data().translated);
							assert(!node->data().translated);
							node->data().translated = true;
						});
	};
	auto reset_translated = [&]() {
		for (const auto& id : ids) {
			graph.findNode(id).value()->data().translated = false;
		}
	};

	// Updating nodes 1+2 should update node 7
	*buffers[0] = 0xa00000b0;
	*buffers[1] = 0x0a00000b;
	translate_node(ids[1]);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[6]).value()->data().translated, false);
	translate_node(ids[0]);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[3]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[5]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[6]).value()->data().translated, true);
	EXPECT_EQ(*buffers[2], 0xa00000b0);
	EXPECT_EQ(*buffers[3], 0xa00000b0 + 1);
	EXPECT_EQ(*buffers[4], 0xa00000b0 + 2);
	EXPECT_EQ(*buffers[5], 0x0a00000b);
	EXPECT_EQ(*buffers[6], ((0xa00000b0 + 2) | 0x0a00000b));

	reset_translated();

	// Now updating nodes 4+6 should update the rest
	*buffers[3] = 0xa00000b0;
	*buffers[5] = 0x0f00000f;
	translate_node(ids[3]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, false);
	EXPECT_EQ(graph.findNode(ids[6]).value()->data().translated, false);
	translate_node(ids[5]);
	EXPECT_EQ(graph.findNode(ids[0]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[1]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[2]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[4]).value()->data().translated, true);
	EXPECT_EQ(graph.findNode(ids[6]).value()->data().translated, true);
	EXPECT_EQ(*buffers[0], 0xa00000b0 + 1);
	EXPECT_EQ(*buffers[1], 0x0f00000f);
	EXPECT_EQ(*buffers[2], 0xa00000b0 + 1);
	EXPECT_EQ(*buffers[4], 0xa00000b0 + 1);
	EXPECT_EQ(*buffers[6], (0xa00000b0 + 1) | 0x0f00000f);
}
