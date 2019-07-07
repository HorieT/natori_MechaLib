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

	struct Node{
		size_t _data_index = 0;
		int16_t _axis_dimension = -1;//����������`����鎟��
		Node* _next[2] = {nullptr};//�q�m�[�h
	};


	std::array<int16_t, Dim> _axis_order = { -1 };
	std::vector<Node> _node;
	Node* _root = nullptr;

	std::vector<std::pair<Key, Data>> _data;

	/*�ċA�I�c���[�\�z�֐�*/
	Node* build_recursive(typename std::vector<Node>::iterator first, typename  std::vector<Node>::iterator last, size_t depth)
	{
		if (first >= last)return nullptr;

		const int16_t  axis = _axis_order.at(depth % Dim);//���̎���
		const size_t mid = (last - first) / 2;//�z��̒����@�����؂�̂�
		auto now_node = std::next(first, mid);//���݂̃m�[�h


		//nth�Ԗڂ�菬�������̂�nth�Ԃ̑O���ɏW�߂�
		std::nth_element(
			first, //first
			now_node, //nth(��C���f�b�N�X)
			last, //last
			[&](Node lhs, Node rhs) {return _data[lhs._data_index].first[axis] < _data[rhs._data_index].first[axis]; });//compare�֐�

		(*now_node)._next[0] = build_recursive(first, now_node, depth + 1);
		(*now_node)._next[1] = build_recursive(std::next(now_node), last, depth + 1);
		(*now_node)._axis_dimension = axis;

		return &*now_node;
	}

	/*�ċA�I�ŋߖT�T��*/
	void NN_search_recursive(const Key& query, Node* node, float& min, size_t& out_index) {
		if (node == nullptr)return;
		//�����m�[�h
		const Key& check_node_key = _data[node->_data_index].first;

		//��������
		const float distance = get_distance(query, check_node_key);
		if (distance < min) {
			min = distance;
			out_index = node->_data_index;
		}
		//�q�m�[�h�̒T��
		const child_size direction = query[node->_axis_dimension] < check_node_key[node->_axis_dimension] ? child_size::SMALL : child_size::LARGE;
		NN_search_recursive(query, node->_next[static_cast<uint8_t>(direction)], min, out_index);

		const float differ = fabsf(static_cast<float>(query[node->_axis_dimension] - check_node_key[node->_axis_dimension]));
		if (differ < min)
			NN_search_recursive(query, node->_next[!static_cast<uint8_t>(direction)], min, out_index);

	}

	/*�_�ԋ�������*/
	inline float get_distance(const Key& point1, const Key& point2) {
		float dist = 0.0f;
		for (size_t i = 0; i < Dim; ++i)
			dist += powf(static_cast<float>(point1[i] - point2[i]), 2.0f);
		return sqrtf(dist);
	}
	inline float get_distance_sq(const Key& point1, const Key& point2) {
		float dist = 0.0f;
		for (size_t i = 0; i < Dim; ++i)
			dist += powf(static_cast<float>(point1[i] - point2[i]), 2.0f);
		return dist;
	}
public:
	kdtree() { std::iota(_axis_order.begin(), _axis_order.end(), 0); }
	~kdtree() {}

	//�f�[�^�z�񂩂�c���[�̍\�z
	void build(std::vector<std::pair<Key, Data>>& data) {
		//���݂̃c���[�̍폜
		clear();

		//�m�[�h���̃������m��
		_node.resize(data.size());
		for (size_t i = 0; i < data.size(); ++i)_node.at(i)._data_index = i;
		_data = data;
		//����������
		std::iota(_axis_order.begin(), _axis_order.end(), 0);

		//�����Ƃ̕��U�𓾂�
		std::array<float, Dim> dispersion;
		for (size_t dim = 0; dim < Dim; ++dim) {
			float average = std::accumulate(
				_data.begin(),
				_data.end(),
				0.0f,
				[=](float acc, std::pair<Key, Data> i) {return static_cast<float>(i.first[dim]) + acc; })
				/ static_cast<float>(_data.size());

			dispersion.at(dim) = std::accumulate(
				_data.begin(),
				_data.end(),
				0.0f,
				[=](float acc, std::pair<Key, Data> i) {return powf(static_cast<float>(i.first[dim]) - average, 2.0f) + acc; })
				/ static_cast<float>(_data.size());
		}
		//���U�̑傫�����Ɏ�����ׂ�
		std::sort(_axis_order.begin(), _axis_order.end(), [&](size_t a, size_t b) {return dispersion.at(a) > dispersion.at(b); });


		//���U�̑傫���������]���Ė؂��\�z
		_root = build_recursive(_node.begin(), _node.end(), 0);
	}

	/*�c���[�̍폜*/
	void clear() {
		_node.clear();
		_data.clear();
	}

	/*�ŋߖT�T��*/
	const std::pair<Key, Data>& NN_search(const Key& query, float& distance, size_t& index) {
		distance = std::numeric_limits<float>::max();
		typename std::vector<Node>::iterator get_node;
		NN_search_recursive(query, _root, distance, index);

		return _data[index];
	}
	/*
	std::vector<std::pair<Key, Data>> kNN_search(const Key& query, size_t num) {

	}*/

	inline size_t size() { return _data.size(); }
	const std::pair<Key, Data>& operator[](size_t index) const& {return _data[index];}
	std::pair<Key, Data>& operator[](size_t index) & {return _data[index];}
	std::pair<Key, Data> operator[](size_t index) const&& {return _data[index];}
};
}
