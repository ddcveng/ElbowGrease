package main_desktop

import game ".."

main :: proc() {
    game.game_init_window()
    game.game_init()

    for game.game_should_run() {
            game.game_update()
    }

    game.game_shutdown()
    game.game_shutdown_window()
}
