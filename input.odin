package jamgame

//import "core:container/queue"
import rl "vendor:raylib"

MoveDirection :: enum {
    Forward, 
    Backward,
    Left,
    Right,
}
MovementSet :: bit_set[MoveDirection]

CameraRotation :: [2]f32

InputActions :: struct {
    movement: MovementSet,
    jump: bool,
    cameraRotation: CameraRotation
}

PollActions :: proc() -> InputActions
{
    actions := InputActions{}

    // Movement
    movement := MovementSet{}
    if rl.IsKeyDown(.W) {
        movement += {.Forward}
    }
    if rl.IsKeyDown(.S) {
        movement += {.Backward}
    }
    if rl.IsKeyDown(.A) {
        movement += {.Left}
    }
    if rl.IsKeyDown(.D) {
        movement += {.Right}
    }

    // Normalize the movement bitset
    if .Forward in movement && .Backward in movement {
        movement -= {.Forward, .Backward}
    }
    if .Left in movement && .Right in movement {
        movement -= {.Left, .Right}
    }

    actions.movement = movement

    // Jumping
    if rl.IsKeyPressed(.SPACE) {
        actions.jump = true
    }

    // Camera rotation
    mouseDelta := rl.GetMouseDelta()
    actions.cameraRotation = CameraRotation(mouseDelta)

    return actions
}

