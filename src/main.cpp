#include <SFML/Graphics.hpp>

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <memory>
#include <variant>

enum class Mode {
    Tiles, Collision, Test
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

struct Tree;

struct Leaf {
    std::vector<std::size_t> lines;

    void push_line(Segment const& line, std::size_t id) {
        std::cout << line.from.x << ", " << line.from.y << " <-> " << line.to.x << ", " << line.to.y << " with #" << id << " added to " << this << '\n';
        lines.push_back(id);
    }

};

struct Node {
    std::variant<Leaf, std::unique_ptr<Tree>> value; 

    void push_line(Segment const& line, std::size_t id);
};

struct Tree {
    Segment vertical_separator;
    Segment horizontal_separator;

    Node top_left;
    Node top_right;
    Node bottom_left;
    Node bottom_right;

    void push_line(Segment const& line, std::size_t id) {
        auto cv = intersection(vertical_separator, line);
        auto ch = intersection(horizontal_separator, line);

        if (cv && ch) {
            if (line.from.x < vertical_separator.from.x) {
                if (line.from.y < horizontal_separator.from.y) {
                    // top left to bottom right
                    if (ch->x < vertical_separator.from.x) { // pass by bottom left
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        top_left.push_line(line1, id);
                        bottom_left.push_line(line2, id);
                        bottom_right.push_line(line3, id);
                    } else if (ch->x > vertical_separator.from.x) { // pass by top right
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        top_left.push_line(line1, id);
                        top_right.push_line(line2, id);
                        bottom_right.push_line(line3, id);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        top_left.push_line(line1, id);
                        bottom_right.push_line(line2, id);
                    }
                } else {
                    // bottom left to top right
                    if (ch->x < vertical_separator.from.x) { // pass by top left
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        bottom_left.push_line(line1, id);
                        top_left.push_line(line2, id);
                        top_right.push_line(line3, id);
                    } else if (ch->x > vertical_separator.from.x) { // pass by bottom right
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        bottom_left.push_line(line1, id);
                        bottom_right.push_line(line2, id);
                        top_right.push_line(line3, id);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        bottom_left.push_line(line1, id);
                        top_right.push_line(line2, id);
                    }
                }
            } else {
                if (line.from.y < horizontal_separator.from.y) {
                    // top right to bottom left
                    if (ch->x < vertical_separator.from.x) { // pass by top left
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        top_right.push_line(line1, id);
                        top_left.push_line(line2, id);
                        bottom_left.push_line(line3, id);
                    } else if (ch->x > vertical_separator.from.x) { // pass by bottom right 
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        top_right.push_line(line1, id);
                        bottom_right.push_line(line2, id);
                        bottom_left.push_line(line3, id);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        top_right.push_line(line1, id);
                        bottom_left.push_line(line2, id);
                    }
                } else {
                    // bottom right to top left
                    if (ch->x < vertical_separator.from.x) { // pass by bottom left
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, *ch };
                        Segment line3{ *ch, line.to };
                        bottom_right.push_line(line1, id);
                        bottom_left.push_line(line2, id);
                        top_left.push_line(line3, id);
                    } else if (ch->x > vertical_separator.from.x) { // pass by top right
                        Segment line1{ line.from, *ch };
                        Segment line2{ *ch, *cv };
                        Segment line3{ *cv, line.to };
                        bottom_right.push_line(line1, id);
                        top_right.push_line(line2, id);
                        top_left.push_line(line3, id);
                    } else { // intersect at the center
                        Segment line1{ line.from, *cv };
                        Segment line2{ *cv, line.to };
                        bottom_right.push_line(line1, id);
                        top_left.push_line(line2, id);
                    }
                }
            }
        } else if (cv) {
            Segment line1{ line.from, *cv };
            Segment line2{ *cv, line.to };

            if (line.from.y < horizontal_separator.from.y) {
                if (line.from.x < vertical_separator.from.x) {
                    top_left.push_line(line1, id);
                    top_right.push_line(line2, id);
                } else {
                    top_left.push_line(line2, id);
                    top_right.push_line(line1, id);
                }
            } else {
                if (line.from.x < vertical_separator.from.x) {
                    bottom_left.push_line(line1, id);
                    bottom_right.push_line(line2, id);
                } else {
                    bottom_left.push_line(line2, id);
                    bottom_right.push_line(line1, id);
                }
            }
        } else if (ch) {
            Segment line1{ line.from, *ch };
            Segment line2{ *ch, line.to };
            if (line.from.x < vertical_separator.from.x) {
                if (line.from.y < horizontal_separator.from.y) {
                    top_left.push_line(line1, id);
                    bottom_left.push_line(line2, id);
                } else {
                    top_left.push_line(line2, id);
                    bottom_left.push_line(line1, id);
                }
            } else {
                if (line.from.y < horizontal_separator.from.y) {
                    top_right.push_line(line1, id);
                    bottom_right.push_line(line2, id);
                } else {
                    top_right.push_line(line2, id);
                    bottom_right.push_line(line1, id);
                }
            }
        } else {
            if (line.from.x < vertical_separator.from.x) {
                if (line.from.y < horizontal_separator.from.y) {
                    top_left.push_line(line, id);
                } else {
                    bottom_left.push_line(line, id);
                }
            } else {
                if (line.from.y < horizontal_separator.from.y) {
                    top_right.push_line(line, id);
                } else {
                    bottom_right.push_line(line, id);
                }
            }
        }
    }
};

void Node::push_line(Segment const& line, std::size_t id) {
    if (auto leaf = std::get_if<Leaf>(&value)) {
        leaf->push_line(line, id);
    } else {
        auto& tree = std::get<std::unique_ptr<Tree>>(value);
        tree->push_line(line, id);
    }
}


struct QuadTree {

    std::vector<Segment> lines;
    std::vector<Point> points;



    void push_line(Segment line) {
        bool from_point_found = false;
        bool to_point_found = false;
        for(auto& p : points) {
            if (!from_point_found && p.position == line.from) {
                p.lines.push_back(lines.size());
                from_point_found = true;
                if (to_point_found) break;
            }

            if (!to_point_found && p.position == line.to) {
                p.lines.push_back(lines.size());
                to_point_found = true;
                if (from_point_found) break;
            }
        }
        lines.push_back(std::move(line));
    }

};


int main(int argc, char** argv) {

    std::unordered_map<sf::Vector2i, sf::Sprite, decltype([] (sf::Vector2i const& p) { return std::size_t(p.x^p.y); })> sprites;
    Mode mode = Mode::Test;

    int const tile_size = 70;
    sf::Vector2i tile_atlas_position{ 0, 0 };
    sf::Vector2i const atlas_size{ 14, 8 };

    bool save_on_exit = false;
    char const* filename = "level0.lvl";
    
    std::vector<std::vector<sf::Vertex>> collision_group_vertices = {{}};
    std::size_t current_group = 0;

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

    Tree tree;
    std::cout << "TOP LEFT IS " << &tree.top_left << '\n';
    std::cout << "TOP RIGHT IS " << &tree.top_right << '\n';
    std::cout << "BOTTOM LEFT IS " << &tree.bottom_left << '\n';
    std::cout << "BOTTOM RIGHT IS " << &tree.bottom_right << '\n';

    tree.vertical_separator = {
        separators[0].position,
        separators[1].position
    };
    tree.horizontal_separator = {
        separators[2].position,
        separators[3].position
    };

    while(window.isOpen()) {
        sf::Event event;
        while(window.pollEvent(event)) {
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::M) {
                    mode = mode == Mode::Tiles ? Mode::Collision : Mode::Tiles;
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
                            collision_group_vertices[current_group].push_back(sf::Vertex(mouse_on_grid, sf::Color::Red));
                        } else if (event.mouseButton.button == sf::Mouse::Button::Right){
                            collision_group_vertices.push_back({});
                            ++current_group;
                        }
                    } else if (event.type == sf::Event::MouseWheelScrolled) {
                        auto delta = event.mouseWheelScroll.delta;
                        std::cout << "DELTA: " << delta << '\n';
                        while(delta > 0 && grid_size < tile_size) {
                            grid_size *= 2;
                            --delta;
                        }
                        while(delta < 0 && grid_size > 4) {
                            grid_size /= 2;
                            ++delta;
                        }
                        std::cout << "GRID: " << grid_size << '\n';
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
                            tree.push_line({ current_line[0].position, current_line[1].position }, 0);
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

                break;
            }
            case Mode::Test: {
                if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                    window.draw(current_line.data(), current_line.size(), sf::PrimitiveType::Lines);
                }
                window.draw(all_lines.data(), all_lines.size(), sf::PrimitiveType::Lines);
                window.draw(separators.data(), separators.size(), sf::PrimitiveType::Lines);
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