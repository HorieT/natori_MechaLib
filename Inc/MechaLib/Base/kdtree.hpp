/*
 * 2019/07/03 Horie
 */
#pragma once

#include <utility>
#include <algorithm>
#include <numeric>
#include <vector>
#include <typeinfo>

namespace Mecha{

/*
 *
 */
template<typename Key, typename Data, size_t Dim = sizeof(Key)>
class kdtree {
private:
	enum class child_size : uint8_t { SMALL = 0U, LARGE = 1U };

	struct Node : public std::pair<Key, Data> {
		int16_t axis_dimension = -1;//分割軸が定義される次元
		typename std::vector<Node>::iterator _next[2];//子ノード

		Node(Key first, Data second) : std::pair<Key, Data>(first, second) {}
		Node(std::pair<Key, Data> data) : std::pair<Key, Data>(data) {}
	};

	std::vector<Node> _node;
	std::array<int16_t, Dim> _axis_order = {-1};
	typename std::vector<Node>::iterator _root;

	/*再帰的ツリー構築関数*/
	typename std::vector<Node>::iterator build_recursive(typename std::vector<Node>::iterator first, typename  std::vector<Node>::iterator last, size_t depth)
	{
		if (first >= last)return last;

		const int16_t  axis = _axis_order.at(depth % Dim);//軸の次元
		const size_t mid = (last - first) / 2;//配列の中央　少数切り捨て
		auto now_node = std::next(first, mid);//現在のノード


		//nth番目より小さいものをnth番の前方に集める
		std::nth_element(
			first, //first
			now_node, //nth(基準インデックス)
			last, //last
			[&](Node lhs, Node rhs) {return lhs.first[axis] < rhs.first[axis]; });//compare関数

		(*now_node)._next[0] = build_recursive(first, now_node, depth + 1);
		(*now_node)._next[1] = (std::next(now_node) == last) ? now_node : build_recursive( std::next(now_node), last, depth + 1);
		(*now_node).axis_dimension = axis;

		return now_node;
	}

	/*再帰的最近傍探索*/
	void NN_search_recursive(const Key& query, typename std::vector<Node>::iterator& it, float& min, typename std::vector<Node>::iterator& out_node) {
		//調査ノード
		const Key& check_node_key = (*it).first;

		//距離測定
		const float distance = get_distance(query, check_node_key);
		if (distance < min) {
			min = distance;
			out_node = it;
		}
		//子ノードの探査
		if ((*it).axis_dimension > -1) {
			const child_size direction = query[(*it).axis_dimension] < check_node_key[(*it).axis_dimension] ? child_size::SMALL : child_size::LARGE;
			if (auto child_it = (*it)._next[static_cast<uint8_t>(direction)]; child_it != it)
				NN_search_recursive(query, child_it, min, out_node);

			const float differ = fabsf(static_cast<float>(query[(*it).axis_dimension] - check_node_key[(*it).axis_dimension]));
			if (differ < min) {
				if (auto child_it = (*it)._next[!static_cast<uint8_t>(direction)]; child_it != it)
					NN_search_recursive(query, child_it, min, out_node);
			}
		}
	}

	/*点間距離測定*/
	inline float get_distance(const Key& point1, const Key& point2){
		float dist = 0.0f;
		for (size_t i = 0; i < Dim; ++i)
			dist += powf(static_cast<float>(point1[i] - point2[i]), 2.0f);
		return sqrtf(dist);
	}
public:
	constexpr kdtree() {}
	~kdtree() {}

	//データ配列からツリーの構築
	constexpr void build(std::vector<std::pair<Key, Data>>& data) {
		//現在のツリーの削除
		clear();

		//ノードのメモリ確保
		_node.reserve(data.size());
		for (auto d : data)_node.push_back(d);
		//軸順初期化
		std::iota(_axis_order.begin(), _axis_order.end(), 0);

		//軸ごとの分散を得る
		std::array<float, Dim> dispersion;
		for (size_t dim = 0; dim < Dim; ++dim) {
			float average = std::accumulate(
				data.begin(),
				data.end(),
				0.0f,
				[=](float acc, std::pair<Key, Data> i) {return static_cast<float>(i.first[dim]) + acc; })
				/ static_cast<float>(data.size());

			dispersion.at(dim) = std::accumulate(
				data.begin(),
				data.end(),
				0.0f,
				[=](float acc, std::pair<Key, Data> i) {return powf(static_cast<float>(i.first[dim]) - average, 2.0f) + acc; })
				/ static_cast<float>(data.size());
		}
		//分散の大きい順に軸を並べる
		std::sort(_axis_order.begin(), _axis_order.end(), [&](size_t a, size_t b) {return dispersion.at(a) > dispersion.at(b); });


		//分散の大きい軸から回転して木を構築
		build_recursive(_node.begin(), _node.end(), 0);

		_root = std::next(_node.begin(), (_node.size()) / 2);
	}

	/*ツリーの削除*/
	void clear() {
		_node.clear();
	}

	/*最近傍探索*/
	std::pair<Key, Data> NN_search(const Key& query, float& distance) {
		distance = std::numeric_limits<float>::max();
		typename std::vector<Node>::iterator get_node;
		NN_search_recursive(query, _root, distance, get_node);

		return std::pair<Key, Data>{(*get_node).first, (*get_node).second};
	}
	/*
	std::vector<std::pair<Key, Data>> kNN_search(const Key& query, size_t num) {

	}*/

	inline size_t size() { return _node.size(); }
	inline std::pair<Key, Data> operator[](size_t indecs) {return std::pair<Key, Data>{_node[indecs].first, _node[indecs].second };}
};
}
