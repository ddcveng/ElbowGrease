package jamgame

//import "core:container/queue"
import rl "vendor:raylib"
import "core:math/linalg"
import "core:fmt"

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
    cameraRotation: CameraRotation,
    interact: bool,
}

poll_actions :: proc{poll_actions_raw, poll_actions_inherit_queuable}

poll_actions_inherit_queuable :: proc(previousActions: InputActions) -> InputActions
{
    actions := poll_actions_raw()
    actions.jump |= previousActions.jump
    actions.interact |= previousActions.interact

    return actions
}

firstMouseDelta := rl.Vector2(0)
mouseMoved := false
firstMouse := true

poll_actions_raw :: proc() -> InputActions
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
    actions.jump = rl.IsKeyPressed(.SPACE)

    // Camera rotation
    mouseDelta := rl.GetMouseDelta()
    // fmt.println("mouse before")
    // fmt.println(mouseDelta)
    // if !mouseMoved {
    //     if firstMouse {
    //         firstMouseDelta = mouseDelta
    //         mouseDelta = rl.Vector2(0)
    //         firstMouse = false
    //     }
    //     else {
    //         diff := mouseDelta - firstMouseDelta
    //         if linalg.length2(diff) > rl.EPSILON {
    //             mouseMoved = true
    //         }
    //         else {
    //             mouseDelta = rl.Vector2(0)
    //         }
    //
    //     }
    // }
    // fmt.println("mouse after")
    // fmt.println(mouseDelta)
    actions.cameraRotation = CameraRotation(mouseDelta)

    // Interact
    actions.interact = rl.IsKeyPressed(.E)

    return actions
}

