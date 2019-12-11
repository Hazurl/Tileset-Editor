#include <SFML/Graphics.hpp>

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <cmath>

enum class Mode {
    Tiles, Collision
};

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
    sf::Vector2i position{ sf::Mouse::getPosition().x / tile_size, sf::Mouse::getPosition().y / tile_size };
    tile_selected.setPosition(position.x * tile_size, position.y * tile_size);

    sf::Vector2i mouse_position = sf::Mouse::getPosition();
    sf::RectangleShape point({4,4});
    point.setFillColor(sf::Color::Red);
    point.setOrigin({2,2});
    float grid_size = tile_size;

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
            }
        }

        window.clear(sf::Color{50, 50, 50});
        for(auto&&[_, spr] : sprites) {
            window.draw(spr);
        }
        for(auto const& grp : collision_group_vertices) {
            if (grp.size() > 1) {
                window.draw(grp.data(), grp.size(), sf::PrimitiveType::LineStrip);
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