#include <SFML/Graphics.hpp>

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <memory>
#include <bitset>
#include <variant>
#include <cassert>

#include <sftk/print/Printer.hpp>

#define SFTK_GIZMO_GLOBAL_PROPERTIES
#include <sftk/gizmo/Gizmo.hpp>

enum class Mode {
    Tiles, Collision, TestQuadtree, TestRay, SIZE
};

struct Segment {
    sf::Vector2f from;
    sf::Vector2f to;
};

struct Point {
    sf::Vector2f position;
    std::vector<std::size_t> lines;
};

float sq_magnitude(sf::Vector2f const& v) {
    return v.x * v.x + v.y * v.y;
}

float magnitude(sf::Vector2f const& v) {
    return std::sqrt(sq_magnitude(v));
}

sf::Vector2f normalize(sf::Vector2f const& v) {
    float mag = magnitude(v);
    return { v.x / mag, v.y / mag }; 
}

bool collinear(sf::Vector2f const& lhs, sf::Vector2f const& rhs) {
    if (lhs.x == 0) {
        return rhs.x == 0;
    }
    float dx = rhs.x / lhs.x;
    return lhs.y * dx == rhs.y;
}

float cross_product(sf::Vector2f const& lhs, sf::Vector2f const& rhs) {
    return lhs.x * rhs.y - rhs.x * lhs.y;
}

std::optional<sf::Vector2f> intersection(Segment const& lhs, Segment const& rhs) {
    auto p = lhs.from;
    auto q = rhs.from;
    auto r = lhs.to - lhs.from; 
    auto s = rhs.to - rhs.from; 

    float rs = cross_product(r, s);
    if (rs == 0) {
        return std::nullopt;
    }

    float qps = cross_product(q - p, s);
    float qpr = cross_product(q - p, r);

    float t = qps / rs;
    float u = qpr / rs;

    if (t < 0 || t > 1 || u < 0 || u > 1) {
        return std::nullopt;
    }

    return p + t * r;
}

std::optional<sf::Vector2f> intersection_with_x(float x, Segment const& lhs) {
    if (lhs.from.x < x && lhs.to.x < x) {
        return std::nullopt;
    }

    if (lhs.from.x > x && lhs.to.x > x) {
        return std::nullopt;
    }

    auto d = lhs.to - lhs.from;
    if(d.x == 0.f) {
        return std::nullopt;
    }

    auto t = (x - lhs.from.x) / d.x;
    auto y = lhs.from.y + t * d.y;
    return sf::Vector2f{ x, y };
}

std::optional<sf::Vector2f> intersection_with_y(float y, Segment const& lhs) {
    if (lhs.from.y < y && lhs.to.y < y) {
        return std::nullopt;
    }

    if (lhs.from.y > y && lhs.to.y > y) {
        return std::nullopt;
    }

    auto d = lhs.to - lhs.from;
    if(d.y == 0.f) {
        return std::nullopt;
    }

    auto t = (y - lhs.from.y) / d.y;
    auto x = lhs.from.x + t * d.x;
    return sf::Vector2f{ x, y };
}

template<typename T, std::size_t N>
struct BlockedLinkedList {
    struct Block {
        std::array<T, N> values;
        std::unique_ptr<Block> next;
    };

    struct Iterator {
        Block* block;
        std::size_t i;

        using value_type = T; 
        using pointer = T*; 
        using reference = T&; 
        using difference_type = void;
        using iterator_category = std::forward_iterator_tag;

        T& operator*() const {
            return block->values[i];
        }

        T* operator->() const {
            return &block->values[i];
        }

        Iterator& operator++() {
            if(i == 0) {
                block = block->next.get();
                i = N;
            }
            --i;
            return *this;
        }

        Iterator operator++(int) const {
            auto it = *this;
            return ++it;
        }

        bool operator==(Iterator const& other) const {
            return other.block == block && other.i == i;
        }

        bool operator!=(Iterator const& other) const {
            return !(*this == other);
        }
    };

    struct ConstIterator {
        Block const* block;
        std::size_t i;

        using value_type = T; 
        using pointer = T const*; 
        using reference = T const&; 
        using difference_type = void;
        using iterator_category = std::forward_iterator_tag;

        T const& operator*() const {
            return block->values[i];
        }

        T const* operator->() const {
            return &block->values[i];
        }

        ConstIterator& operator++() {
            if(i == 0) {
                block = block->next.get();
                i = N;
            }
            --i;
            return *this;
        }

        ConstIterator operator++(int) const {
            auto it = *this;
            return ++it;
        }

        bool operator==(ConstIterator const& other) const {
            return other.block == block && other.i == i;
        }

        bool operator!=(ConstIterator const& other) const {
            return !(*this == other);
        }
    };

    static_assert(N >= 1);

    std::unique_ptr<Block> root = nullptr;
    std::size_t size = 0;

    bool is_filled() const {
        return size % N == 0;
    }

    void push(T const& v) {
        auto idx = size % N;

        if (idx == 0) {
            auto block = std::make_unique<Block>();
            block->next = std::move(root);
            root = std::move(block);
        }

        root->values[idx] = v;
        ++size;
    }

    Iterator begin() {
        return Iterator{ root.get(), (size + N - 1) % N };
    }

    Iterator end() {
        return Iterator{ nullptr, N - 1 };
    }

    ConstIterator begin() const {
        return ConstIterator{ root.get(), (size + N - 1) % N };
    }

    ConstIterator end() const {
        return ConstIterator{ nullptr, N - 1 };
    }

    ConstIterator cbegin() const {
        return begin();
    }

    ConstIterator cend() const {
        return end();
    }
};  

template<typename T, typename I = std::size_t>
struct Recycler {
    static constexpr I no_more_free_indices = std::numeric_limits<std::size_t>::max();
    
    static constexpr I chunk_size_order = 5;
    static constexpr I chunk_size = 1 << chunk_size_order;
    static constexpr I chunk_index_mask = chunk_size - 1;

    struct Block {
        union Value {
            Value() : next_free_index_offset{ 0 } {};
            ~Value() {};
            I next_free_index_offset;
            T value;
        };

        std::unique_ptr<Value[]> values;
        std::bitset<chunk_size> valid;

        Block() : values(std::make_unique<Value[]>(chunk_size)) {};
        Block(nullptr_t) {};

        Block(Block const&) = delete;
        Block& operator=(Block const&) = delete;

        Block(Block&& b) {
            std::swap(b.values, values);
            std::swap(b.valid, valid);
        }

        Block& operator=(Block&& b) {
            std::swap(b.values, values);
            std::swap(b.valid, valid);
        }

        template<typename...Args>
        void construct(std::size_t i, Args&&...args) {
            assert(!valid[i]);
            new (&values[i].value) T(std::forward<Args>(args)...);
            valid[i] = true;
        }

        void destroy(std::size_t i) {
            value_at(i).~T();
            valid[i] = false;
        }

        T& value_at(std::size_t i) {
            assert(valid[i]);
            return values[i].value;
        }

        T const& value_at(std::size_t i) const {
            assert(valid[i]);
            return values[i].value;
        }

        I& free_index_offset_at(std::size_t i) {
            assert(!valid[i]);
            return values[i].next_free_index_offset;
        }

        I const& free_index_offset_at(std::size_t i) const {
            assert(!valid[i]);
            return values[i].next_free_index_offset;
        }

        ~Block() {
            for(std::size_t i = 0; i < chunk_size; ++i) {
                if (valid[i]) {
                    destroy(i);
                }
            }
        }
    };

    static_assert(sizeof(I) <= sizeof(T));

    std::vector<Block> blocks;
    I free_index = 0;

    bool empty() const {
        return blocks.empty();
    }

    static std::pair<I, I> decompose(I idx) {
        return { idx >> chunk_size_order, idx & chunk_index_mask };
    }

    T const& operator[](I idx) const {
        auto[block_idx, in_block_idx] = decompose(idx);
        return blocks[block_idx].value_at(in_block_idx);
    }

    T& operator[](I idx) {
        auto[block_idx, in_block_idx] = decompose(idx);
        return blocks[block_idx].value_at(in_block_idx);
    }

    I capacity() const {
        return blocks.size() << chunk_size_order;
    }

    I push(T const& t) {
        return emplace(t);
    }

    I push(T&& t) {
        return emplace(std::move(t));
    }

    template<typename...Args>
    I emplace(Args&&...args) {
        auto const idx = free_index;

        if (idx >= capacity()) {
            blocks.emplace_back();
        }

        auto[block_idx, in_block_idx] = decompose(idx);

        free_index = blocks[block_idx].free_index_offset_at(in_block_idx) + 1 + idx;
        blocks[block_idx].construct(in_block_idx, std::forward<Args>(args)...);

        return idx;
    }

    void pop(I idx) {
        auto[block_idx, in_block_idx] = decompose(idx);
        blocks[block_idx].destroy(in_block_idx);
        blocks[block_idx].free_index_offset_at(in_block_idx) = free_index - 1 - idx;
        free_index = idx;
    }
};

using separation_point_list_t = std::vector<std::pair<float, bool>>;

template<float sf::Vector2f::* M>
void push_segment_member(separation_point_list_t& points, Segment const& s) {
    points.push_back({ s.from.*M, s.from.*M < s.to.*M });
    points.push_back({ s.to.*M, s.from.*M >= s.to.*M });
}

void push_segment_x(separation_point_list_t& points, Segment const& s) {
    return push_segment_member<&sf::Vector2f::x>(points, s);
}

void push_segment_y(separation_point_list_t& points, Segment const& s) {
    return push_segment_member<&sf::Vector2f::y>(points, s);
}

float separation_heuristic(int lhs, int rhs, int n) {
    // lhs: number of elements *enterely* on the left hand side
    // rhs: number of elements *enterely* on the right hand side
    // n: total number of elements

    // Transform an value in [0, n] into an inverse 'V' functions in [0, n]:
    //
    // n   / \              .
    //    /   \             .
    // 0 /     \            .
    //  0   n   0 
    // So the more the value is close to n/2, the higher the value is
    auto const df = [n] (int x) {
        return n - std::abs(2*x - n);
    };

    auto df_lhs = df(lhs);
    auto df_rhs = df(rhs);
    auto sum = lhs + rhs; // scale the heuristic based on the number of elements correctly split

    // note: the result will be 0 if lhs or rhs is 0
    // this is legitimate as a separation that doesn't separate any elements isn't useful
    return df_lhs * df_rhs * sum;
}

struct BestSeparation {
    static constexpr int min_heuristic = 0;

    int heuristic = min_heuristic;
    std::size_t index = 0;
    float room_size = std::numeric_limits<float>::min();
    float lhs, rhs;
};

BestSeparation find_best_separation(std::vector<std::pair<float, bool>> points) {
    // Segments = 2 points
    assert(points.size() % 2 == 0);

    // Sort points based on their component
    // The second member mean if this is the first point of the segment
    // A segment (a <-> b) should be inserted as (a, true) *then* (b, false)
    std::sort(std::begin(points), std::end(points), [] (auto const& lhs, auto const& rhs) {
        return lhs.first < rhs.first;
    });

    int const n = points.size() >> 1; // number of segments

    int lhs = 0; // numbers of segments on the left hand side
    int rhs = n; // numbers of segments on the right hand side

    BestSeparation best;

    for(std::size_t i = 0; i < points.size() - 1;) {
        // Accumulate all the points on the same axis together, we don't want to separate them
        float x = points[i].first;
        do {
            if(!points[i++].second) 
                ++lhs;
            else
                --rhs;
        } while(i < points.size() && points[i].first == x);

        int const current_heurisitic = separation_heuristic(lhs, rhs, n);
        float current_room = points[i].first - points[i - 1].first;
        // The room size is only taken into account when there's a similar heurisitic
        if (current_heurisitic > best.heuristic 
        || (current_heurisitic == best.heuristic && current_room > best.room_size)) {
            best = {
                current_heurisitic,
                i - 1,
                current_room   
            };
        }
    }

    if (best.heuristic > BestSeparation::min_heuristic) {
        // Heuristic of the last index should return 0, thus not end up in this branch
        assert(best.index + 1 < points.size());

        best.lhs = points[best.index].first;
        best.rhs = points[best.index + 1].first;
    }

    return best;
}

std::optional<float> compute_best_separation(std::vector<std::pair<float, bool>> points) {
    auto best = find_best_separation(std::move(points));

    // No separation exists such that at least one segment can be separated from another
    // In other words, they are all overlapping
    if (best.heuristic == BestSeparation::min_heuristic) {
        return std::nullopt;
    }

    return (best.lhs + best.rhs) / 2.f;
}

enum class SeparationAxis {
    Vertical, Horizontal
};

std::optional<std::pair<SeparationAxis, float>> compute_best_separation(std::vector<Segment> const& segments) {
    separation_point_list_t xs;
    separation_point_list_t ys;

    for(auto const& s : segments) {
        push_segment_x(xs, s);
        push_segment_y(ys, s);
    }

    auto const best_x = find_best_separation(std::move(xs));
    auto const best_y = find_best_separation(std::move(ys));

    // No separation exists such that at least one segment can be separated from another
    // In other words, they are all overlapping
    if (best_x.heuristic == BestSeparation::min_heuristic && best_y.heuristic == BestSeparation::min_heuristic) {
        std::cout << "No separation available\n";
        return std::nullopt;
    }

    if (best_x.heuristic < best_y.heuristic) {
        std::cout << "Best Separation on Y " << (best_y.lhs + best_y.rhs) / 2.f << '\n';
        return std::make_pair(SeparationAxis::Horizontal, (best_y.lhs + best_y.rhs) / 2.f);
    }

    std::cout << "Best Separation on X " << (best_x.lhs + best_x.rhs) / 2.f << '\n';
    return std::make_pair(SeparationAxis::Vertical, (best_x.lhs + best_x.rhs) / 2.f);
}

struct QuadTreeNode;

struct QuadTreeLeaf {
    BlockedLinkedList<std::size_t, 4> values;

    bool is_filled() const {
        return values.size != 0 && values.is_filled();
    }

    void push(Recycler<Segment>& segments, Segment const& segment) {
        values.push(segments.push(segment));
    }

};

struct QuadTreeInternalNodeGrid {
    /*
        0: top left
        1: top right
        2: bottom left
        3: bottom rigth
    */
    std::array<QuadTreeNode*, 4> quadrants;

    float x, y;

    inline QuadTreeNode* top_left() { return quadrants[0]; }
    inline QuadTreeNode* top_right() { return quadrants[1]; }
    inline QuadTreeNode* bottom_left() { return quadrants[2]; }
    inline QuadTreeNode* bottom_right() { return quadrants[3]; }

    inline QuadTreeNode const* top_left() const { return quadrants[0]; }
    inline QuadTreeNode const* top_right() const { return quadrants[1]; }
    inline QuadTreeNode const* bottom_left() const { return quadrants[2]; }
    inline QuadTreeNode const* bottom_right() const { return quadrants[3]; }

    inline std::size_t quadrant_id(sf::Vector2f const& p) const {
        if (p.x < x  && p.y < y ) return 0; // top left
        if (p.x >= x && p.y < y ) return 1; // top right
        if (p.x < x  && p.y >= y) return 2; // bottom left
        return 3;                           // bottom right
    }

    std::array<std::optional<Segment>, 4> separate_segment(Segment const& seg) const;
};

struct QuadTreeInternalNodeHorizontal {
    /*
        0: top
        1: bottom
    */
    std::array<QuadTreeNode*, 2> quadrants;

    float y;

    QuadTreeInternalNodeHorizontal(float y_) : quadrants{ nullptr }, y{ y_ } {}

    inline QuadTreeNode* top() { return quadrants[0]; }
    inline QuadTreeNode* bottom() { return quadrants[1]; }

    inline QuadTreeNode const* top() const { return quadrants[0]; }
    inline QuadTreeNode const* bottom() const { return quadrants[1]; }

    inline std::size_t quadrant_id(sf::Vector2f const& p) const {
        return p.y < y ? 0 : 1;
    }

    std::array<std::optional<Segment>, 2> separate_segment(Segment const& seg) const;
};

struct QuadTreeInternalNodeVertical {
    /*
        0: left
        1: right
    */
    std::array<QuadTreeNode*, 2> quadrants;

    float x;

    QuadTreeInternalNodeVertical(float x_) : quadrants{ nullptr }, x{ x_ } {}

    inline QuadTreeNode* left() { return quadrants[0]; }
    inline QuadTreeNode* right() { return quadrants[1]; }

    inline QuadTreeNode const* left() const { return quadrants[0]; }
    inline QuadTreeNode const* right() const { return quadrants[1]; }

    inline std::size_t quadrant_id(sf::Vector2f const& p) const {
        return p.x < x ? 0 : 1;
    }

    std::array<std::optional<Segment>, 2> separate_segment(Segment const& seg) const;
};

template<typename Strategy>
struct QuadTreeInternalNode : Strategy {
    
    template<typename...Args>
    QuadTreeInternalNode(Args&&...args) : Strategy(std::forward<Args>(args)...) {}

    template<typename F>
    void containing(Segment const& seg, F f) const;
    QuadTreeLeaf const* containing_point(sf::Vector2f const& p) const;

    void push(Recycler<QuadTreeNode>& nodes, unsigned char max_depth_rem, Recycler<Segment>& segments, Segment const& segment);
};


struct QuadTreeNode {
    std::variant<QuadTreeLeaf, QuadTreeInternalNode<QuadTreeInternalNodeVertical>, QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>> node;
    unsigned char max_depth_rem;

    bool is_leaf() const {
        return std::holds_alternative<QuadTreeLeaf>(node);
    }

    bool is_vertical_node() const {
        return std::holds_alternative<QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>>(node);
    }

    bool is_horizontal_node() const {
        return std::holds_alternative<QuadTreeInternalNode<QuadTreeInternalNodeVertical>>(node);
    }

    QuadTreeLeaf& as_leaf() {
        assert(is_leaf());
        return std::get<QuadTreeLeaf>(node);
    }

    QuadTreeLeaf const& as_leaf() const {
        assert(is_leaf());
        return std::get<QuadTreeLeaf>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>& as_vertical_node() {
        assert(is_vertical_node());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeHorizontal> const& as_vertical_node() const {
        assert(is_vertical_node());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeVertical>& as_horizontal_node() {
        assert(is_horizontal_node());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeVertical>>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeVertical> const& as_horizontal_node() const {
        assert(is_horizontal_node());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeVertical>>(node);
    }

    template<typename F>
    void containing(Segment const& seg, F f) const {
        if (is_leaf()) {
            f(as_leaf());
        } else {
            if (is_vertical_node()) {
                as_vertical_node().containing(seg, f);
            } else {
                as_horizontal_node().containing(seg, f);
            }
        }
    }

    QuadTreeLeaf const* containing_point(sf::Vector2f const& p) const {
        if (is_leaf()) {
            return &as_leaf();
        } else {
            if (is_vertical_node()) {
                return as_vertical_node().containing_point(p);
            } else {
                return as_horizontal_node().containing_point(p);
            }
        }
    }

    void push(Recycler<QuadTreeNode>& nodes, Recycler<Segment>& segments, Segment const& segment) {
        if (is_leaf()) {
            auto& leaf = as_leaf();

            std::cout << "Slice ? " << leaf.values.size << " && " << int(max_depth_rem) << " => " << (max_depth_rem != 0 && leaf.is_filled()) << '\n';
            if (max_depth_rem != 0 && leaf.is_filled()) {
                auto transfert_to = [&] (auto new_node) {
                    for(auto i : leaf.values) {
                        auto seg = segments[i];
                        segments.pop(i);
                        new_node.push(nodes, max_depth_rem, segments, seg);
                    }
                    return new_node;
                };

                std::vector<Segment> all_segments;
                all_segments.reserve(leaf.values.size + 1); // segments in the leaf + the segment being pushed

                for(auto s : leaf.values) {
                    all_segments.push_back(segments[s]);
                }
                all_segments.push_back(segment);


                auto best_separation = compute_best_separation(all_segments);

                if (best_separation) {
                    auto[axis, value] = *best_separation;
                    if (axis == SeparationAxis::Horizontal) {
                        node = transfert_to(QuadTreeInternalNode<QuadTreeInternalNodeHorizontal>{ value });
                    } else {
                        node = transfert_to(QuadTreeInternalNode<QuadTreeInternalNodeVertical>{ value });
                    }
                }
                // else keep the leaf
            }

            if (is_leaf()) { // no good separation where found...
                std::cout << "### Finally Push (" << segment.from.x << ", " << segment.from.y << ") => (" << segment.to.x << ", " << segment.to.y << ")\n";
                as_leaf().push(segments, segment);
                return;
            }

        }

        if (is_vertical_node()) {
            as_vertical_node().push(nodes, max_depth_rem, segments, segment);
        } else {
            as_horizontal_node().push(nodes, max_depth_rem, segments, segment);
        }
    }
};
/*
struct QuadTreeNode {
    std::variant<QuadTreeLeaf, QuadTreeInternalNode<QuadTreeInternalNodeGrid>> node;
    unsigned char max_depth_rem;

    bool is_leaf() const {
        return std::holds_alternative<QuadTreeLeaf>(node);
    }

    QuadTreeLeaf& as_leaf() {
        assert(is_leaf());
        return std::get<QuadTreeLeaf>(node);
    }

    QuadTreeLeaf const& as_leaf() const {
        assert(is_leaf());
        return std::get<QuadTreeLeaf>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeGrid>& as_node() {
        assert(!is_leaf());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeGrid>>(node);
    }

    QuadTreeInternalNode<QuadTreeInternalNodeGrid> const& as_node() const {
        assert(!is_leaf());
        return std::get<QuadTreeInternalNode<QuadTreeInternalNodeGrid>>(node);
    }

    template<typename F>
    void containing(Segment const& seg, F f) const {
        if (is_leaf()) {
            f(as_leaf());
        } else {
            as_node().containing(seg, f);
        }
    }

    QuadTreeLeaf const* containing_point(sf::Vector2f const& p) const {
        if (is_leaf()) {
            return &as_leaf();
        } else {
            return as_node().containing_point(p);
        }
    }

    void push(Recycler<QuadTreeNode>& nodes, Recycler<Segment> const& segments, Segment const& seg, std::size_t idx) {
        if (is_leaf()) {
            if (max_depth_rem == 0) {
                std::cerr << "Nope\n";
                return;
                //throw std::runtime_error("#");
            }
            auto& leaf = as_leaf();

            if (leaf.used < std::size(leaf.values)) {
                leaf.values[leaf.used++] = idx;
                return;
            }
            QuadTreeInternalNode<QuadTreeInternalNodeGrid> new_node{ 0, 0, 0, 0 };

            float x = 0;
            float y = 0;
            for(std::size_t i = 0; i < leaf.used; ++i) {
                auto const& s = segments[leaf.values[i]];
                x += s.from.x + s.to.x;
                y += s.from.y + s.to.y;
            }

            x /= leaf.used * 2;
            y /= leaf.used * 2;

            new_node.x = x;
            new_node.y = y;

            for(std::size_t i = 0; i < leaf.used; ++i) {
                new_node.push(nodes, segments, max_depth_rem, segments[leaf.values[i]], leaf.values[i]);
            }

            node = new_node;
        }

        as_node().push(nodes, segments, max_depth_rem, seg, idx);
    }
};
*/
template<typename S>
template<typename F>
void QuadTreeInternalNode<S>::containing(Segment const& seg, F f) const {
    auto parts = S::separate_segment(seg);
    for(std::size_t q = 0; q < std::size(parts); ++q) {
        if (parts[q]) {
            S::quadrants[q]->containing(seg, f);
        }
    }
}

template<typename S>
QuadTreeLeaf const* QuadTreeInternalNode<S>::containing_point(sf::Vector2f const& p) const {
    auto q = S::quadrants[S::quadrant_id(p)];
    if (q) {
        return q->containing_point(p);
    }

    return nullptr;
}

template<typename S>
void QuadTreeInternalNode<S>::push(Recycler<QuadTreeNode>& nodes, unsigned char max_depth_rem, Recycler<Segment>& segments, Segment const& segment) {
    auto parts = S::separate_segment(segment);
    for(std::size_t q = 0; q < std::size(parts); ++q) {
        if (parts[q]) {
            if (!S::quadrants[q]) {
                S::quadrants[q] = &nodes[nodes.push({ QuadTreeLeaf{}, static_cast<unsigned char>(max_depth_rem - 1) })];
            }
            S::quadrants[q]->push(nodes, segments, *parts[q]);
        }
    }
}

std::array<std::optional<Segment>, 4> QuadTreeInternalNodeGrid::separate_segment(Segment const& seg) const {
    std::array<std::optional<Segment>, 4> parts{ std::nullopt };

    auto from = seg.from;
    auto to = seg.to;

    auto q_from = quadrant_id(from);
    auto q_to = quadrant_id(to);

    if (q_to < q_from) {
        std::swap(q_from, q_to);
        std::swap(from, to);
    }

    // Same quadrant
    if (q_from == q_to) {
        parts[q_from] = seg;
        return parts;
    }

    // top left <-> top right or bottom left <-> bottom right
    if (q_from + 1 == q_to && (q_from % 2) == 0) {
        auto cv = intersection_with_x(x, seg);
        assert(cv);
        parts[q_from] = Segment{ from, *cv };
        parts[q_to] = Segment{ *cv, to };
        return parts;
    }

    // top left <-> bottom left or top right <-> bottom right
    if (q_from + 2 == q_to) {
        auto ch = intersection_with_y(y, seg);
        assert(ch);
        parts[q_from] = Segment{ from, *ch };
        parts[q_to] = Segment{ *ch, to };
        return parts;
    }

    // top left <-> bottom right
    if (q_from == 0) {
        auto cv = intersection_with_x(x, seg);
        auto ch = intersection_with_y(y, seg);

        assert(q_to == 3);
        assert(cv && ch);

        if (cv == ch) { // Pass by the center
            parts[q_from] = Segment{ from, *ch };
            parts[q_to] = Segment{ *ch, to };
        } else if(ch->x < x) { // Pass by bottom left quadrant
            assert(cv->y >= y);

            parts[q_from] = Segment{ from, *ch };
            parts[2] = Segment{ *ch, *cv };
            parts[q_to] = Segment{ *cv, to };
        } else { // Pass by top right quadrant
            assert(cv->y < y);

            parts[q_from] = Segment{ from, *cv };
            parts[1] = Segment{ *cv, *ch };
            parts[q_to] = Segment{ *ch, to };
        }

        return parts;
    }

    // top right <-> bottom left
    auto cv = intersection_with_x(x, seg);
    auto ch = intersection_with_y(y, seg);

    assert(q_from == 1 && q_to == 2);
    assert(cv && ch);

    if (cv == ch) { // Pass by the center
        parts[q_from] = Segment{ from, *ch };
        parts[q_to] = Segment{ *ch, to };
    } else if(ch->x < x) { // Pass by top left quadrant
        assert(cv->y < y);

        parts[q_from] = Segment{ from, *ch };
        parts[0] = Segment{ *ch, *cv };
        parts[q_to] = Segment{ *cv, to };
    } else { // Pass by bottom right quadrant
        assert(cv->y >= y);

        parts[q_from] = Segment{ from, *cv };
        parts[3] = Segment{ *cv, *ch };
        parts[q_to] = Segment{ *ch, to };
    }

    return parts;
}

std::array<std::optional<Segment>, 2> QuadTreeInternalNodeHorizontal::separate_segment(Segment const& seg) const {
    std::array<std::optional<Segment>, 2> parts{ std::nullopt };

    auto const q_from = quadrant_id(seg.from);
    auto const q_to = quadrant_id(seg.to);

    // Same quadrant
    if (q_from == q_to) {
        parts[q_from] = seg;
        return parts;
    }

    auto intersection = intersection_with_y(y, seg);
    assert(intersection);
    parts[q_from] = Segment{ seg.from, *intersection };
    parts[q_to] = Segment{ seg.to, *intersection };

    return parts;
}

std::array<std::optional<Segment>, 2> QuadTreeInternalNodeVertical::separate_segment(Segment const& seg) const {
    std::array<std::optional<Segment>, 2> parts{ std::nullopt };

    auto const q_from = quadrant_id(seg.from);
    auto const q_to = quadrant_id(seg.to);

    // Same quadrant
    if (q_from == q_to) {
        parts[q_from] = seg;
        return parts;
    }

    auto intersection = intersection_with_x(x, seg);
    assert(intersection);
    parts[q_from] = Segment{ seg.from, *intersection };
    parts[q_to] = Segment{ seg.to, *intersection };

    return parts;
}

struct Quadtree {
    Recycler<Segment> segments;
    Recycler<QuadTreeNode> nodes;

    QuadTreeNode& get_root() {
        assert(!nodes.empty());
        return nodes[0];
    }

    QuadTreeNode const& get_root() const {
        assert(!nodes.empty());
        return nodes[0];
    }

    void push(Segment const& s) {
        std::cout << std::string(40, '=') << '\n';
        std::cout << "\tPush (" << s.from.x << ", " << s.from.y << ") => (" << s.to.x << ", " << s.to.y << ")\n";
        std::cout << std::string(40, '=') << '\n';
        get_root().push(nodes, segments, s);
    }

    template<typename F>
    void intersections(Segment const& s, F&& f) const {
        get_root().containing(s, [&] (QuadTreeLeaf const& leaf) {
            for(auto value : leaf.values) {
                if (auto p = intersection(s, segments[value])) {
                    f(segments[value], *p);
                }
            }
        });
    }

    template<typename F>
    void close_to(sf::Vector2f p, F&& f) const {
        auto leaf = get_root().containing_point(p);
        if (leaf) {
            for(auto s : leaf->values) {
                f(segments[s]);
            }
        }
    }
};

struct Tree;

struct Leaf {
    std::vector<Segment> lines;

    void push_line(Segment const& line) {
        lines.push_back(line);
    }

};

struct Node {
    unsigned char max_depth = 10;
    std::variant<Leaf, std::unique_ptr<Tree>> value = Leaf{}; 

    void push_line(Segment const& line);
};

struct Tree {
    sf::Vector2f separators;

    Node top_left;
    Node top_right;
    Node bottom_left;
    Node bottom_right;

    void push_line(Segment const& line) {
        auto cv = intersection_with_x(separators.x, line);
        auto ch = intersection_with_y(separators.y, line);

        if (cv && ch) {
            if (line.from.x < separators.x) {
                if (line.from.y < separators.y) {
                    // top left to bottom right
                    if (ch->x < separators.x) { // pass by bottom left
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        top_left.push_line(line1);
                        bottom_left.push_line(line2);
                        bottom_right.push_line(line3);
                    } else if (ch->x > separators.x) { // pass by top right
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        top_left.push_line(line1);
                        top_right.push_line(line2);
                        bottom_right.push_line(line3);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        top_left.push_line(line1);
                        bottom_right.push_line(line2);
                    }
                } else {
                    // bottom left to top right
                    if (ch->x < separators.x) { // pass by top left
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        bottom_left.push_line(line1);
                        top_left.push_line(line2);
                        top_right.push_line(line3);
                    } else if (ch->x > separators.x) { // pass by bottom right
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        bottom_left.push_line(line1);
                        bottom_right.push_line(line2);
                        top_right.push_line(line3);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        bottom_left.push_line(line1);
                        top_right.push_line(line2);
                    }
                }
            } else {
                if (line.from.y < separators.y) {
                    // top right to bottom left
                    if (ch->x < separators.x) { // pass by top left
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        top_right.push_line(line1);
                        top_left.push_line(line2);
                        bottom_left.push_line(line3);
                    } else if (ch->x > separators.x) { // pass by bottom right 
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        top_right.push_line(line1);
                        bottom_right.push_line(line2);
                        bottom_left.push_line(line3);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        top_right.push_line(line1);
                        bottom_left.push_line(line2);
                    }
                } else {
                    // bottom right to top left
                    if (ch->x < separators.x) { // pass by bottom left
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        bottom_right.push_line(line1);
                        bottom_left.push_line(line2);
                        top_left.push_line(line3);
                    } else if (ch->x > separators.x) { // pass by top right
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        bottom_right.push_line(line1);
                        top_right.push_line(line2);
                        top_left.push_line(line3);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        bottom_right.push_line(line1);
                        top_left.push_line(line2);
                    }
                }
            }
        } else if (cv) {
            Segment line1{ line.from, *cv };
            Segment line2{ *cv, line.to };

            if (line.from.y < separators.y) {
                if (line.from.x < separators.x) {
                    top_left.push_line(line1);
                    top_right.push_line(line2);
                } else {
                    top_left.push_line(line2);
                    top_right.push_line(line1);
                }
            } else {
                if (line.from.x < separators.x) {
                    bottom_left.push_line(line1);
                    bottom_right.push_line(line2);
                } else {
                    bottom_left.push_line(line2);
                    bottom_right.push_line(line1);
                }
            }
        } else if (ch) {
            Segment line1{ line.from, *ch };
            Segment line2{ *ch, line.to };
            if (line.from.x < separators.x) {
                if (line.from.y < separators.y) {
                    top_left.push_line(line1);
                    bottom_left.push_line(line2);
                } else {
                    top_left.push_line(line2);
                    bottom_left.push_line(line1);
                }
            } else {
                if (line.from.y < separators.y) {
                    top_right.push_line(line1);
                    bottom_right.push_line(line2);
                } else {
                    top_right.push_line(line2);
                    bottom_right.push_line(line1);
                }
            }
        } else {
            if (line.from.x < separators.x) {
                if (line.from.y < separators.y) {
                    top_left.push_line(line);
                } else {
                    bottom_left.push_line(line);
                }
            } else {
                if (line.from.y < separators.y) {
                    top_right.push_line(line);
                } else {
                    bottom_right.push_line(line);
                }
            }
        }
    }
};

void Node::push_line(Segment const& line) {
    if (auto leaf = std::get_if<Leaf>(&value)) {
        if (leaf->lines.size() >= 4 && max_depth > 0) {
            auto tree = std::make_unique<Tree>();
            tree->separators = line.from + line.to;
            for(auto const& l : leaf->lines) {
                tree->separators += l.from + l.to;
            }

            tree->separators.x /= leaf->lines.size() * 2 + 2;
            tree->separators.y /= leaf->lines.size() * 2 + 2;

            for(auto const& l : leaf->lines) {
                tree->push_line(l);
            }
            tree->push_line(line);
            tree->top_left.max_depth = max_depth - 1;
            tree->top_right.max_depth = max_depth - 1;
            tree->bottom_left.max_depth = max_depth - 1;
            tree->bottom_right.max_depth = max_depth - 1;
            value = std::move(tree);
        } else {
            leaf->push_line(line);
        }
    } else {
        auto& tree = std::get<std::unique_ptr<Tree>>(value);
        tree->push_line(line);
    }
}

void walk(sf::RenderTarget& target, sf::FloatRect bounds, Node const& node, std::size_t depth = 0) {
    static std::array colors {
        sf::Color::Magenta,
        sf::Color::Cyan,
        sf::Color::Yellow,
    };
    if (std::holds_alternative<Leaf>(node.value)) return;

    auto const& tree = *std::get<std::unique_ptr<Tree>>(node.value);

    std::array separators{
        sf::Vertex({ tree.separators.x, bounds.top }, colors[depth % colors.size()]),
        sf::Vertex({ tree.separators.x, bounds.top + bounds.height }, colors[depth % colors.size()]),
        sf::Vertex({ bounds.left, tree.separators.y }, colors[depth % colors.size()]),
        sf::Vertex({ bounds.left + bounds.width, tree.separators.y }, colors[depth % colors.size()])
    };

    target.draw(separators.data(), separators.size(), sf::PrimitiveType::Lines);

    float bx = bounds.left;
    float sx = tree.separators.x;
    float ex = bounds.left + bounds.width;

    float by = bounds.top;
    float sy = tree.separators.y;
    float ey = bounds.top + bounds.height;

    sf::FloatRect top_left_bounds(
        bx, by, 
        sx - bx, sy - by);
    sf::FloatRect top_right_bounds(
        sx, by, 
        ex - sx, sy - by);
    sf::FloatRect bottom_left_bounds(
        bx, sy, 
        sx - bx, ey - sy);
    sf::FloatRect bottom_right_bounds(
        sx, sy, 
        ex - sx, ey - sy);

    walk(target, top_left_bounds, tree.top_left, depth + 1);
    walk(target, top_right_bounds, tree.top_right, depth + 1);
    walk(target, bottom_left_bounds, tree.bottom_left, depth + 1);
    walk(target, bottom_right_bounds, tree.bottom_right, depth + 1);
}

void walk(sf::RenderTarget& target, sf::FloatRect bounds, Quadtree const& quadtree, QuadTreeNode const& node, std::size_t depth = 0) {
    static std::array colors {
        sf::Color::Magenta,
        sf::Color::Cyan,
        sf::Color::Yellow,
    };
    if (node.is_leaf()) return;

    if (node.is_horizontal_node()) {
        auto const& internal_node = node.as_horizontal_node();
        std::array separators{
            sf::Vertex({ internal_node.x, bounds.top }, colors[depth % colors.size()]),
            sf::Vertex({ internal_node.x, bounds.top + bounds.height }, colors[depth % colors.size()])
        };

        target.draw(separators.data(), separators.size(), sf::PrimitiveType::Lines);

        sf::FloatRect left_bounds(
            bounds.left, bounds.top, 
            internal_node.x - bounds.left, bounds.height);
        sf::FloatRect right_bounds(
            internal_node.x, bounds.top, 
            bounds.left + bounds.width - internal_node.x, bounds.height);

        if(internal_node.left()) walk(target, left_bounds, quadtree, *internal_node.left(), depth + 1);
        if(internal_node.left()) walk(target, right_bounds, quadtree, *internal_node.right(), depth + 1);
    } else {
        auto const& internal_node = node.as_vertical_node();
        std::array separators{
            sf::Vertex({ bounds.left, internal_node.y }, colors[depth % colors.size()]),
            sf::Vertex({ bounds.left + bounds.width, internal_node.y }, colors[depth % colors.size()])
        };

        target.draw(separators.data(), separators.size(), sf::PrimitiveType::Lines);

        sf::FloatRect left_bounds(
            bounds.left, bounds.top, 
            bounds.width, internal_node.y - bounds.top);
        sf::FloatRect right_bounds(
            bounds.left, internal_node.y, 
            bounds.width, bounds.top + bounds.height - internal_node.y);

        if(internal_node.top()) walk(target, left_bounds, quadtree, *internal_node.top(), depth + 1);
        if(internal_node.bottom()) walk(target, right_bounds, quadtree, *internal_node.bottom(), depth + 1);
    }
}

Leaf& find_leaf_with_point(Node& node, sf::Vector2f const& p) {
    if (auto leaf = std::get_if<Leaf>(&node.value)) {
        return *leaf;
    }

    auto& tree = *std::get<std::unique_ptr<Tree>>(node.value);

    if (tree.separators.x > p.x) {
        if (tree.separators.y > p.y) {
            return find_leaf_with_point(tree.top_left, p);
        } else {
            return find_leaf_with_point(tree.bottom_left, p);
        }
    } else {
        if (tree.separators.y > p.y) {
            return find_leaf_with_point(tree.top_right, p);
        } else {
            return find_leaf_with_point(tree.bottom_right, p);
        }
    }
}


int main(int argc, char** argv) {

    std::unordered_map<sf::Vector2i, sf::Sprite, decltype([] (sf::Vector2i const& p) { return std::size_t(p.x^p.y); })> sprites;
    Mode mode = Mode::Tiles;

    int const tile_size = 70;
    sf::Vector2i tile_atlas_position{ 0, 0 };
    sf::Vector2i const atlas_size{ 14, 8 };

    bool save_on_exit = false;
    char const* filename = "level0.lvl";
    
    std::vector<std::vector<sf::Vertex>> collision_group_vertices = {{}};
    std::size_t current_group = 0;

    Quadtree new_quadtree;
    new_quadtree.nodes.push({ QuadTreeLeaf{}, 10 });

    if (argc >= 2) {
        bool already_loaded = false;
        for(int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--load" && !already_loaded) {
                already_loaded = true;
                std::cout << "Load file\n";
                std::ifstream file(filename);

                int x, y, id;
                while(file.peek() != 'c') {
                    file >> x >> y >> id;
                    auto& spr = sprites.emplace(sf::Vector2i{ x, y }, sf::Sprite()).first->second;
                    spr.setTextureRect(sf::IntRect(sf::Vector2i{ id % atlas_size.x, id / atlas_size.x } * tile_size, { tile_size, tile_size }));
                    spr.setPosition(x * tile_size, y * tile_size);
                    while(file.peek() == '\n') file.get();
                }

                file.get(); // 'c'

                float fx, fy;
                while(file >> fx >> fy) {
                    if (collision_group_vertices[current_group].size() >= 1) {
                        auto p0 = collision_group_vertices[current_group].back().position;
                        auto p1 = sf::Vector2f{ fx, fy };
                        new_quadtree.push(Segment{ p0, p1 });
                    }

                    collision_group_vertices[current_group].push_back(sf::Vertex({fx, fy}, sf::Color::Red));
                    if (file.peek() == '\n') {
                        collision_group_vertices.push_back({});
                        ++current_group;
                    }
                }

            } else if (arg == "--save") {
                std::cout << "Will be saved on exit\n";
                save_on_exit = true;
            }
        }
    }

    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML Light test", sf::Style::Default);

    sf::Texture tileset;
    if (!tileset.loadFromFile("assets/tiles/Tilesheet/platformerPack_industrial_tilesheet.png")) {
        return 1;
    }
    sf::Sprite tile_selected(tileset, sf::IntRect(tile_atlas_position * tile_size, { tile_size, tile_size }));
    for(auto&s : sprites) {
        s.second.setTexture(tileset);
    }
    sf::Vector2i position{ sf::Mouse::getPosition(window).x / tile_size, sf::Mouse::getPosition(window).y / tile_size };
    tile_selected.setPosition(position.x * tile_size, position.y * tile_size);

    sf::Vector2i mouse_position = sf::Mouse::getPosition(window);
    sf::RectangleShape point({4,4});
    point.setFillColor(sf::Color::Red);
    point.setOrigin({2,2});
    float grid_size = tile_size;

    std::array current_line{
        sf::Vertex({}, sf::Color::Red),
        sf::Vertex({}, sf::Color::Red)
    };

    std::array separators{
        sf::Vertex({ window.getSize().x / 2.f, 0 }, sf::Color::Magenta),
        sf::Vertex({ window.getSize().x / 2.f, window.getSize().y }, sf::Color::Magenta),
        sf::Vertex({ 0, window.getSize().y / 2.f }, sf::Color::Magenta),
        sf::Vertex({ window.getSize().x, window.getSize().y / 2.f }, sf::Color::Magenta)
    };

    std::vector<sf::Vertex> all_lines;

    Quadtree test_quadtree;
    test_quadtree.nodes.push({ QuadTreeLeaf{}, 10 });

    bool test_ray_rotating = false;
    Segment test_ray{
        { mouse_position.x, mouse_position.y },
        { mouse_position.x + 1000.f, mouse_position.y}
    };

    sf::Clock clock;

    while(window.isOpen()) {
        sf::Event event;
        float const dt = clock.restart().asSeconds();
        while(window.pollEvent(event)) {
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::M) {
                    mode = static_cast<Mode>((static_cast<std::underlying_type_t<Mode>>(mode) + 1) % static_cast<std::underlying_type_t<Mode>>(Mode::SIZE));
                    continue;
                }
            }

            switch(mode) {
/////////////////////////////////////////////////////////////////////////////////////
// MODE TILES
/////////////////////////////////////////////////////////////////////////////////////
                case Mode::Tiles: {
                    if (event.type == sf::Event::Closed) {
                        window.close();
                    } else if (event.type == sf::Event::KeyPressed) {
                        if (event.key.code == sf::Keyboard::Up) {
                            tile_atlas_position.y = (tile_atlas_position.y + atlas_size.y - 1) % atlas_size.y;
                        } else if (event.key.code == sf::Keyboard::Down) {
                            tile_atlas_position.y = (tile_atlas_position.y + 1) % atlas_size.y;
                        } else if (event.key.code == sf::Keyboard::Left) {
                            tile_atlas_position.x = (tile_atlas_position.x + atlas_size.x - 1) % atlas_size.x;
                        } else if (event.key.code == sf::Keyboard::Right) {
                            tile_atlas_position.x = (tile_atlas_position.x + 1) % atlas_size.x;
                        }

                        tile_selected.setTextureRect(sf::IntRect(tile_atlas_position * tile_size, { tile_size, tile_size }));
                    } else if (event.type == sf::Event::MouseWheelScrolled) {
                        auto delta = event.mouseWheelScroll.delta;
                        while (delta < 0) {
                            delta += atlas_size.x * atlas_size.y;
                        }
                        tile_atlas_position.x += delta;
                        tile_atlas_position.y = (tile_atlas_position.y + tile_atlas_position.x / atlas_size.x) % atlas_size.y;
                        tile_atlas_position.x %= atlas_size.x;
                        tile_selected.setTextureRect(sf::IntRect(tile_atlas_position * tile_size, { tile_size, tile_size }));
                    } else if (event.type == sf::Event::MouseButtonPressed) {
                        if (event.mouseButton.button == sf::Mouse::Button::Left) {
                            sprites.emplace(position, tile_selected).first->second.setPosition(position.x * tile_size, position.y * tile_size);
                        } else if (event.mouseButton.button == sf::Mouse::Button::Right){
                            sprites.erase(position);
                        }
                    } else if (event.type == sf::Event::MouseMoved) {
                        position = {
                            event.mouseMove.x / tile_size,
                            event.mouseMove.y / tile_size
                        };
                        auto real_position = position * tile_size;
                        tile_selected.setPosition(float(real_position.x), float(real_position.y));
                    }

                    break;
                }

/////////////////////////////////////////////////////////////////////////////////////
// MODE COLLISION
/////////////////////////////////////////////////////////////////////////////////////
                case Mode::Collision: {
                    if (event.type == sf::Event::Closed) {
                        window.close();
                    } else if (event.type == sf::Event::KeyPressed) {
                    } else if (event.type == sf::Event::MouseButtonPressed) {
                        if (event.mouseButton.button == sf::Mouse::Button::Left) {
                            sf::Vector2f mouse_on_grid{ 
                                std::round(mouse_position.x / grid_size) * grid_size, 
                                std::round(mouse_position.y / grid_size) * grid_size
                            };
                            if (collision_group_vertices[current_group].size() >= 1) {
                                auto p0 = collision_group_vertices[current_group].back().position;
                                new_quadtree.push(Segment{ p0, mouse_on_grid });
                            }
                            collision_group_vertices[current_group].push_back(sf::Vertex(mouse_on_grid, sf::Color::Red));
                        } else if (event.mouseButton.button == sf::Mouse::Button::Right){
                            collision_group_vertices.push_back({});
                            ++current_group;
                        }
                    } else if (event.type == sf::Event::MouseWheelScrolled) {
                        auto delta = event.mouseWheelScroll.delta;
                        while(delta > 0 && grid_size < tile_size) {
                            grid_size *= 2;
                            --delta;
                        }
                        while(delta < 0 && grid_size > 4) {
                            grid_size /= 2;
                            ++delta;
                        }
                    } else if (event.type == sf::Event::MouseMoved) {
                        mouse_position = { event.mouseMove.x, event.mouseMove.y };
                    }
                    break;
                }

/////////////////////////////////////////////////////////////////////////////////////
// MODE TEST QUADTREE
/////////////////////////////////////////////////////////////////////////////////////
                case Mode::TestQuadtree: {
                    if (event.type == sf::Event::Closed) {
                        window.close();
                    } else if (event.type == sf::Event::MouseButtonPressed) {
                        if (event.mouseButton.button == sf::Mouse::Button::Left) {
                            current_line[0].position = { event.mouseButton.x, event.mouseButton.y };
                            current_line[1].position = { event.mouseButton.x, event.mouseButton.y };
                        }
                    } else if (event.type == sf::Event::MouseButtonReleased) {
                        if (event.mouseButton.button == sf::Mouse::Button::Left) {
                            current_line[1].position = { event.mouseButton.x, event.mouseButton.y };
                            all_lines.emplace_back(current_line[0]).color = sf::Color::Blue;
                            all_lines.emplace_back(current_line[1]).color = sf::Color::Blue;
                            test_quadtree.push({ current_line[0].position, current_line[1].position });
                        }
                    } else if (event.type == sf::Event::MouseWheelScrolled) {
                    } else if (event.type == sf::Event::MouseMoved) {
                        current_line[1].position = { event.mouseMove.x, event.mouseMove.y };
                    }
                    break;
                }

/////////////////////////////////////////////////////////////////////////////////////
// MODE TEST RAY
/////////////////////////////////////////////////////////////////////////////////////
                case Mode::TestRay: {
                    if (event.type == sf::Event::Closed) {
                        window.close();
                    } else if (event.type == sf::Event::MouseMoved) {
                        sf::Vector2f mouse{ event.mouseMove.x, event.mouseMove.y };
                        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                            auto dir = mouse - test_ray.from;
                            if (dir != sf::Vector2f{}) {
                                test_ray.to = test_ray.from + normalize(dir) * 1000.f;
                            }
                        } else {
                            auto offset = test_ray.to - test_ray.from;
                            test_ray.from = mouse;
                            test_ray.to = mouse + offset;
                        }
                    } else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::P) {
                        test_ray_rotating = !test_ray_rotating;
                    }
                    break;
                }
            }
        }

        window.clear(sf::Color{50, 50, 50});

        switch(mode) {
            case Mode::Tiles: {
                for(auto&&[_, spr] : sprites) {
                    window.draw(spr);
                }
                window.draw(tile_selected);
                break;
            }
            case Mode::Collision: {
                for(auto&&[_, spr] : sprites) {
                    auto old_color = spr.getColor();
                    auto new_color = old_color; new_color.a /= 4.f;

                    spr.setColor(new_color);
                    window.draw(spr);
                    spr.setColor(old_color);
                }

                for(auto const& grp : collision_group_vertices) {
                    if (grp.size() > 1) {
                        window.draw(grp.data(), grp.size(), sf::PrimitiveType::LineStrip);
                    }
                }

                sf::Vector2f mouse_on_grid{ 
                    std::round(mouse_position.x / grid_size) * grid_size, 
                    std::round(mouse_position.y / grid_size) * grid_size
                };
                if (!collision_group_vertices[current_group].empty()) {
                    std::array line{ collision_group_vertices[current_group].back(), sf::Vertex(mouse_on_grid, sf::Color::Red) };
                    window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
                }

                point.setPosition(mouse_on_grid);
                window.draw(point);

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::I)) {
                    walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), new_quadtree, new_quadtree.get_root());
                    std::vector<sf::Vertex> vertices;
                    new_quadtree.close_to(window.mapPixelToCoords(sf::Mouse::getPosition(window)), [&vertices] (auto const& segment) {
                        vertices.emplace_back(segment.from, sf::Color::Green);
                        vertices.emplace_back(segment.to, sf::Color::Green);
                    });

                    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                }

                break;
            }

            case Mode::TestQuadtree: {
                if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                    window.draw(current_line.data(), current_line.size(), sf::PrimitiveType::Lines);
                }
                window.draw(all_lines.data(), all_lines.size(), sf::PrimitiveType::Lines);
                
                walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), test_quadtree, test_quadtree.get_root());

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::I)) {
                    std::vector<sf::Vertex> vertices;
                    test_quadtree.close_to(window.mapPixelToCoords(sf::Mouse::getPosition(window)), [&] (Segment const& s) {
                        vertices.emplace_back(s.from, sf::Color::Green);
                        vertices.emplace_back(s.to, sf::Color::Green);
                    });
                    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                }
                break;
            }

            case Mode::TestRay: {
                if (test_ray_rotating) {
                    auto dir = normalize(test_ray.to - test_ray.from);
                    auto angle = std::atan2(dir.y, dir.x);
                    angle += dt * M_PI_4;
                    if (angle > M_PI * 2.f) {
                        angle -= M_PI * 2.f;
                    }

                    auto new_dir = sf::Vector2f(std::cos(angle), std::sin(angle));
                    test_ray.to = test_ray.from + new_dir * 1000.f;
                }

                window.draw(all_lines.data(), all_lines.size(), sf::PrimitiveType::Lines);

                std::array ray_vertices{
                    sf::Vertex(test_ray.from, sf::Color::White),
                    sf::Vertex(test_ray.to, sf::Color::White)
                };

                window.draw(ray_vertices.data(), ray_vertices.size(), sf::PrimitiveType::Lines);

                test_quadtree.intersections(test_ray, [&] (auto const& segment, auto const& point) {
                    //sftk::draw_line(window.mapCoordsToPixel(segment.from), window.mapCoordsToPixel(segment.to));
                    //sftk::draw_point(window.mapCoordsToPixel(point));
                });

                break;
            }
        }

        window.display();
    }

    if (save_on_exit) {
        std::ofstream file (filename);
        std::cout << "Saving...\n";
        for(auto const& s : sprites) {
            auto rect = s.second.getTextureRect();
            file << s.first.x << " " << s.first.y << " " << rect.left / tile_size + rect.top / tile_size * atlas_size.x << "\n";
        }
        file << "c\n";
        for(auto const& grp : collision_group_vertices) {
            for(auto const& v : grp) {
                file << " " << v.position.x << " " << v.position.y;
            }
            file << '\n';
        }
        std::cout << "Done\n";
    }
}