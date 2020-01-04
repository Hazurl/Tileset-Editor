#include <SFML/Graphics.hpp>

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <memory>
#include <variant>
#include <cassert>

enum class Mode {
    Tiles, Collision, Test, SIZE
};

struct Segment {
    sf::Vector2f from;
    sf::Vector2f to;
};

struct Point {
    sf::Vector2f position;
    std::vector<std::size_t> lines;
};

float sqrt_magnitude(sf::Vector2f const& v) {
    return v.x * v.x + v.y + v.y;
}

float magnitude(sf::Vector2f const& v) {
    return std::sqrt(sqrt_magnitude(v));
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

template<typename T, typename I = std::size_t>
struct Recycler {
    union Value {
        Value() : next_free_index{} {};
        I next_free_index;
        T value;
    };

    static constexpr I no_more_free_indices = std::numeric_limits<std::size_t>::max();
    
    static constexpr I chunk_size_order = 5;
    static constexpr I chunk_size = 1 << chunk_size_order;
    static constexpr I chunk_index_mask = chunk_size - 1;

    static_assert(sizeof(I) <= sizeof(T));

    std::vector<std::unique_ptr<Value[]>> values;
    I free_index = 0;

    Value const& value_at(I i) const {
        return values[i >> chunk_size_order][i & chunk_index_mask];
    }

    Value& value_at(I i) {
        return values[i >> chunk_size_order][i & chunk_index_mask];
    }

    T const& operator[](I i) const {
        return value_at(i).value;
    }

    T& operator[](I i) {
        return value_at(i).value;
    }

    I capacity() const {
        return values.size() << chunk_size_order;
    }

    void push_new_block() {
        I i = capacity();
        auto& new_block = values.emplace_back(std::make_unique<Value[]>(chunk_size));
        for(std::size_t j = 0; j < chunk_size; ++j) {
            std::cout << "  " << ((values.size() - 1) << chunk_size_order | j) << " -> " << i + j + 1 << '\n';
            new_block[j].next_free_index = i + j + 1;
        }
    }

    I push(T const& t) {
        auto const idx = free_index;

        std::cout << "\nPush\n";
        std::cout << "  idx = " << idx << '\n';
        std::cout << "  capacity = " << capacity() << '\n';

        if (idx >= capacity()) {
            push_new_block();
            std::cout << "  new capacity = " << capacity() << '\n';
        }

        auto& value = value_at(idx);
        free_index = value.next_free_index;
        value.value = t;

        std::cout << "  free_index = " << free_index << '\n';

        return idx;
    }

    void pop(I i) {
        std::cout << "  " << i << " -> " << free_index << '\n';
        std::cout << "  free_index = " << i << '\n';
        value_at(i).next_free_index = free_index;
        free_index = i;
    }
};

std::optional<float> compute_best_separation(std::vector<std::pair<float, bool>> points) {
    std::sort(std::begin(points), std::end(points), [] (auto const& lhs, auto const& rhs) {
        return lhs.first < rhs.first;
    });

    int const n = points.size() >> 1;
    int entering = 0;
    int exiting = 0;

    int min = n << 1; // worst possible result
    std::size_t min_index = 0;

    for(std::size_t i = 0; i < points.size() - 1;) {
        float x = points[i].first;
        do {
            if(points[i].second) {
                entering += 2;
            } else {
                exiting += 2;
            }

            ++i;
        } while(i < points.size() && points[i].first == x);

        int const m = std::abs(n - entering) + std::abs(n - exiting);
        if (m < min) {
            min = m;
            min_index = i-1;
        }
    }

    if (min == n << 1) {
        return std::nullopt;
    }

    assert(min_index + 1 < points.size());

    return (points[min_index].first + points[min_index + 1].first) / 2.f;
}

struct QuadTreeNode;

struct QuadTreeLeaf {
    std::array<std::size_t, 4> values;
    unsigned char used;
};

struct QuadTreeInternalNode {
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

    template<typename F>
    void containing(Segment const& seg, F f) const;
    QuadTreeLeaf const* containing_point(sf::Vector2f const& p) const;

    void push(Recycler<QuadTreeNode>& nodes, Recycler<Segment> const& segments, unsigned char max_depth_rem, Segment const& seg, std::size_t idx);

    std::array<std::optional<Segment>, 4> separate_segment(Segment const& seg) const;
};

struct QuadTreeInternalNodeVerticalSlice {
    /*
        0: top
        1: bottom
    */
    std::array<QuadTreeNode*, 2> quadrants;

    float y;

    inline QuadTreeNode* left() { return quadrants[0]; }
    inline QuadTreeNode* right() { return quadrants[1]; }

    inline QuadTreeNode const* left() const { return quadrants[0]; }
    inline QuadTreeNode const* right() const { return quadrants[1]; }

    inline std::size_t quadrant_id(sf::Vector2f const& p) const {
        return p.y < y ? 0 : 1;
    }

    template<typename F>
    void containing(Segment const& seg, F f) const;
    QuadTreeLeaf const* containing_point(sf::Vector2f const& p) const;

    void push(Recycler<QuadTreeNode>& nodes, Recycler<Segment> const& segments, unsigned char max_depth_rem, Segment const& seg, std::size_t idx);

    std::array<std::optional<Segment>, 2> separate_segment(Segment const& seg) const;
};

struct QuadTreeNode {
    std::variant<QuadTreeLeaf, QuadTreeInternalNode> node;
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

    QuadTreeInternalNode& as_node() {
        assert(!is_leaf());
        return std::get<QuadTreeInternalNode>(node);
    }

    QuadTreeInternalNode const& as_node() const {
        assert(!is_leaf());
        return std::get<QuadTreeInternalNode>(node);
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
            QuadTreeInternalNode new_node{ 0, 0, 0, 0 };

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

template<typename F>
void QuadTreeInternalNode::containing(Segment const& seg, F f) const {
    auto parts = separate_segment(seg);
    for(std::size_t q = 0; q < std::size(parts); ++q) {
        if (parts[q]) {
            quadrants[q]->containing(seg, f);
        }
    }
}

QuadTreeLeaf const* QuadTreeInternalNode::containing_point(sf::Vector2f const& p) const {
    auto q = quadrants[quadrant_id(p)];
    if (q) {
        return q->containing_point(p);
    }

    return nullptr;
}

void QuadTreeInternalNode::push(Recycler<QuadTreeNode>& nodes, Recycler<Segment> const& segments, unsigned char max_depth_rem, Segment const& seg, std::size_t idx) {
    auto parts = separate_segment(seg);
    for(std::size_t q = 0; q < std::size(parts); ++q) {
        if (parts[q]) {
            if (!quadrants[q]) {
                quadrants[q] = &nodes[nodes.push({ QuadTreeLeaf{ {}, 0 }, static_cast<unsigned char>(max_depth_rem - 1) })];
            }
            quadrants[q]->push(nodes, segments, *parts[q], idx);
        }
    }
}
/*
std::array<size_t const*, 4> QuadTreeInternalNode::get_nodes(Segment const& seg) const {
    auto this_mut = const_cast<QuadTreeInternalNode*>(this);
    auto nodes = this_mut->get_nodes(seg);
    std::array<size_t const*, 4> const_nodes;
    std::copy(std::begin(nodes), std::end(nodes), std::begin(const_nodes));
    return const_nodes;
}
*/
std::array<std::optional<Segment>, 4> QuadTreeInternalNode::separate_segment(Segment const& seg) const {
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
/*
std::array<size_t*, 4> QuadTreeInternalNode::get_nodes(Segment const& seg) {
    auto cv = intersection_with_x(x, seg);
    auto ch = intersection_with_y(y, seg);

    bool on_top_left = false;
    bool on_top_right = false;
    bool on_bottom_left = false;
    bool on_bottom_right = false;

    if (cv && ch) {
        if (seg.from.x < x) {
            if (seg.from.y < y) {
                // top left to bottom right
                if (ch->x < x) { // pass by bottom left
                    on_top_left = true;
                    on_bottom_left = true;
                    on_bottom_right = true;
                } else if (ch->x > x) { // pass by top right
                    on_top_left = true;
                    on_top_right = true;
                    on_bottom_right = true;
                } else { // intersect at the center
                    on_top_left = true;
                    on_bottom_right = true;
                }
            } else {
                // bottom left to top right
                if (ch->x < x) { // pass by top left
                    on_bottom_left = true;
                    on_top_left = true;
                    on_top_right = true;
                } else if (ch->x > x) { // pass by bottom right
                    on_bottom_left = true;
                    on_bottom_right = true;
                    on_top_right = true;
                } else { // intersect at the center
                    on_bottom_left= true;
                    on_top_right = true;
                }
            }
        } else {
            if (seg.from.y < y) {
                // top right to bottom left
                if (ch->x < x) { // pass by top left
                    on_top_right = true;
                    on_top_left = true;
                    on_bottom_left = true;
                } else if (ch->x > x) { // pass by bottom right 
                    on_top_right = true;
                    on_bottom_right = true;
                    on_bottom_left = true;
                } else { // intersect at the center
                    on_top_right = true;
                    on_bottom_left = true;
                }
            } else {
                // bottom right to top left
                if (ch->x < x) { // pass by bottom left
                    on_bottom_right = true;
                    on_bottom_left = true;
                    on_top_left = true;
                } else if (ch->x > x) { // pass by top right
                    on_bottom_right = true;
                    on_top_right = true;
                    on_top_left = true;
                } else { // intersect at the center
                    on_bottom_right = true;
                    on_top_left = true;
                }
            }
        }
    } else if (cv) {
        if (seg.from.y < y) {
            on_top_left = true;
            on_top_right = true;
        } else {
            on_bottom_left = true;
            on_bottom_right = true;
        }
    } else if (ch) {
        if (seg.from.x < x) {
            on_top_left = true;
            on_bottom_left = true;
        } else {
            on_top_right = true;
            on_bottom_right = true;
        }
    } else {
        if (seg.from.x < x) {
            if (seg.from.y < y) {
                on_top_left = true;
            } else {
                on_bottom_left = true;
            }
        } else {
            if (seg.from.y < y) {
                on_top_right = true;
            } else {
                on_bottom_right = true;
            }
        }
    }

    std::array<size_t*, 4> nodes{ nullptr };
    std::size_t i = 0;
    if (on_top_left) nodes[i++] = &top_left;
    if (on_top_right) nodes[i++] = &top_right;
    if (on_bottom_left) nodes[i++] = &bottom_left;
    if (on_bottom_right) nodes[i++] = &bottom_right;

    return nodes;
}
*/
struct Quadtree {
    Recycler<Segment> segments;
    Recycler<QuadTreeNode> nodes;

    QuadTreeNode& get_root() {
        assert(!nodes.values.empty());
        return nodes[0];
    }

    QuadTreeNode const& get_root() const {
        assert(!nodes.values.empty());
        return nodes[0];
    }

    void push(Segment const& s) {
        auto idx = segments.push(s);
        get_root().push(nodes, segments, s, idx);
    }

    template<typename F>
    void intersections(Segment const& s, F&& f) const {
        get_root().containing(s, [&] (QuadTreeLeaf const& leaf) {
            for(std::size_t i = 0; i < leaf.used; ++i) {
                if (auto p = intersection(s, segments[leaf.values[i]])) {
                    f(*p);
                }
            }
        });
    }

    template<typename F>
    void close_to(sf::Vector2f p, F&& f) const {
        auto leaf = get_root().containing_point(p);
        if (leaf) {
            for(std::size_t i = 0; i < leaf->used; ++i) {
                f(segments[leaf->values[i]]);
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

    auto const& internal_node = node.as_node();

    std::array separators{
        sf::Vertex({ internal_node.x, bounds.top }, colors[depth % colors.size()]),
        sf::Vertex({ internal_node.x, bounds.top + bounds.height }, colors[depth % colors.size()]),
        sf::Vertex({ bounds.left, internal_node.y }, colors[depth % colors.size()]),
        sf::Vertex({ bounds.left + bounds.width, internal_node.y }, colors[depth % colors.size()])
    };

    target.draw(separators.data(), separators.size(), sf::PrimitiveType::Lines);

    float bx = bounds.left;
    float sx = internal_node.x;
    float ex = bounds.left + bounds.width;

    float by = bounds.top;
    float sy = internal_node.y;
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

    if(internal_node.top_left()) walk(target, top_left_bounds, quadtree, *internal_node.top_left(), depth + 1);
    if(internal_node.top_right()) walk(target, top_right_bounds, quadtree,*internal_node.top_right(), depth + 1);
    if(internal_node.bottom_left()) walk(target, bottom_left_bounds, quadtree,*internal_node.bottom_left(), depth + 1);
    if(internal_node.bottom_right()) walk(target, bottom_right_bounds, quadtree, *internal_node.bottom_right(), depth + 1);
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
    Node collision_quadtree;

    Quadtree new_quadtree;
    new_quadtree.nodes.push({ QuadTreeLeaf{ {}, 0 }, 10 });

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
                        collision_quadtree.push_line(Segment{ p0, p1 });
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

    Node test_quadtree;
    Quadtree new_test_quadtree;
    new_test_quadtree.nodes.push({ QuadTreeLeaf{ {}, 0 }, 10 });

    while(window.isOpen()) {
        sf::Event event;
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
                                collision_quadtree.push_line(Segment{ p0, mouse_on_grid });
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
// MODE TEST
/////////////////////////////////////////////////////////////////////////////////////
                case Mode::Test: {
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
                            test_quadtree.push_line({ current_line[0].position, current_line[1].position });
                            new_test_quadtree.push({ current_line[0].position, current_line[1].position });
                        }
                    } else if (event.type == sf::Event::MouseWheelScrolled) {
                    } else if (event.type == sf::Event::MouseMoved) {
                        current_line[1].position = { event.mouseMove.x, event.mouseMove.y };
                    }
                    break;
                }
            }
        }

        window.clear(sf::Color{50, 50, 50});
        if (mode != Mode::Test) {
            for(auto&&[_, spr] : sprites) {
                window.draw(spr);
            }
            for(auto const& grp : collision_group_vertices) {
                if (grp.size() > 1) {
                    window.draw(grp.data(), grp.size(), sf::PrimitiveType::LineStrip);
                }
            }
        }

        switch(mode) {
            case Mode::Tiles: {
                window.draw(tile_selected);
                break;
            }
            case Mode::Collision: {
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
                    walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), collision_quadtree);
                    auto& leaf = find_leaf_with_point(collision_quadtree, window.mapPixelToCoords(sf::Mouse::getPosition(window)));
                    std::vector<sf::Vertex> vertices(leaf.lines.size() * 2, sf::Vertex({}, sf::Color::Green));
                    auto it = std::begin(vertices);
                    for(auto const& l : leaf.lines) {
                        (it++)->position = l.from;
                        (it++)->position = l.to;
                    }
                    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::O)) {
                    walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), new_quadtree, new_quadtree.get_root());
                    //auto& leaf = find_leaf_with_point(collision_quadtree, window.mapPixelToCoords(sf::Mouse::getPosition(window)));
                    //std::vector<sf::Vertex> vertices(leaf.lines.size() * 2, sf::Vertex({}, sf::Color::Green));
                    //auto it = std::begin(vertices);
                    //for(auto const& l : leaf.lines) {
                    //    (it++)->position = l.from;
                    //    (it++)->position = l.to;
                    //}
                    //window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                }

                break;
            }
            case Mode::Test: {
                /*
                if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                    window.draw(current_line.data(), current_line.size(), sf::PrimitiveType::Lines);
                }
                window.draw(all_lines.data(), all_lines.size(), sf::PrimitiveType::Lines);
                walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), test_quadtree);

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::I)) {
                    auto& leaf = find_leaf_with_point(test_quadtree, window.mapPixelToCoords(sf::Mouse::getPosition(window)));
                    std::vector<sf::Vertex> vertices(leaf.lines.size() * 2, sf::Vertex({}, sf::Color::Green));
                    auto it = std::begin(vertices);
                    for(auto const& l : leaf.lines) {
                        (it++)->position = l.from;
                        (it++)->position = l.to;
                    }
                    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                }*/

                if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                    window.draw(current_line.data(), current_line.size(), sf::PrimitiveType::Lines);
                }
                window.draw(all_lines.data(), all_lines.size(), sf::PrimitiveType::Lines);
                walk(window, sf::FloatRect(0, 0, window.getSize().x, window.getSize().y), new_test_quadtree, new_test_quadtree.get_root());

                if (sf::Keyboard::isKeyPressed(sf::Keyboard::I)) {
                    std::vector<sf::Vertex> vertices;
                    new_test_quadtree.close_to(window.mapPixelToCoords(sf::Mouse::getPosition(window)), [&] (Segment const& s) {
                        vertices.emplace_back(s.from, sf::Color::Green);
                        vertices.emplace_back(s.to, sf::Color::Green);
                    });
                    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::Lines);
                }
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