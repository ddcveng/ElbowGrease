package example

import rl "vendor:raylib"
import "core:fmt"
import "core:strings"
import "core:math"

FPS :: 60 // Frames per second
TPS :: 30 // Simulation tics per seconds
DT :: 1.0 / TPS // Delta time for the simulation

LEVEL_WIDTH :: 10
LEVEL_HEIGHT :: 10

TILE_SIZE :: 3.0

LEVEL_POSITION_X_START :: -LEVEL_WIDTH * TILE_SIZE / 2.0
LEVEL_POSITION_Z_START :: -LEVEL_HEIGHT * TILE_SIZE / 2.0

KEYS_NEEDED :: 3

LevelSymbol :: enum u8 {
    Void = ' ',
    Spawn = 'S',
    Wall = 'x',
    Key = 'k',
    Goal = 'G',
}

LEVEL :: [LEVEL_WIDTH][LEVEL_HEIGHT]u8 {
    {' ','x','x','x','x', 'x', 'x', 'x', 'x', ' '},
    {'x','k',' ','x',' ', ' ', ' ', ' ', ' ', 'x'},
    {'x','x',' ','x','k', 'x', ' ', 'x', ' ', 'x'},
    {'x','x',' ','x','x', 'x', ' ', 'x', 'x', 'x'},
    {'x',' ',' ',' ','k', 'x', ' ', 'x', ' ', 'x'},
    {'x',' ',' ','x',' ', ' ', ' ', 'x', ' ', 'x'},
    {'x',' ','x',' ','x', ' ', 'x', ' ', ' ', 'x'},
    {'x',' ','x',' ',' ', ' ', 'x', ' ', ' ', 'x'},
    {'x','S','x',' ',' ', ' ', ' ', ' ', 'G', 'x'},
    {' ','x','x','x','x', 'x', 'x', 'x', 'x', ' '},
}

PLAYER_INITIAL_POSITION :: rl.Vector3{0.0, 2.0, 0.0}
CAMERA_DISTANCE :: f32(6)
CAMERA_INITIAL_ROTATION :: f32(0)
CAMERA_HEIGHT_DIFFERENCE :: f32(1.5)

pos2LevelIndex :: proc(position: rl.Vector3) -> (int, int) {
    xInx := int(math.round((position.x - LEVEL_POSITION_X_START) / TILE_SIZE))
    zInx := int(math.round((position.z - LEVEL_POSITION_Z_START) / TILE_SIZE))

    return zInx, xInx
}

Pivot :: enum {
    center = 0,
    bottom = 1,
}

adjust_y_pos :: proc(y_pos: f32, scale: f32, pivot := Pivot.center) -> f32 {
    switch (pivot) {
    case .center: return y_pos
    case .bottom: return y_pos + scale / 2.0
    }

    return y_pos
}

PlayerTransform :: struct {
    position: rl.Vector3, // implicitly pivot.center
    rotation: f32, // This is currenlty in degreees, since the rendering code takes it in degrees...
    // most of the calculations require this to be radians though so it kinda sucks
    grounded: bool
}

// Distance should only change when we would intersect a wall. IN this case zoom the camera in a little to avoid it.
// rotation will change when player rotates the camera independently from rotating the player character, when the player
// character rotates, this stays the same (the camera follows players rotation)
PlayerFollowingCamera :: struct {
    distance: f32,
    rotation: f32, // in radians, relative to players direction. 0 means the camera faces the same direction as the player. 
}

GameState :: struct {
    playerTransform: PlayerTransform,
    cameraData: PlayerFollowingCamera,
}

// Hm.. keeping this up to date as the state grows will be tricky,
// can I maybe generate this? or say that if the actual leafs of  the state graph are vectors or scalars,
// I can automatically inspect the type and interpolate them ??
interpolate_states :: proc(s0: ^GameState, s1: ^GameState, alpha: f32) -> GameState {
    newPlayerPos := s0.playerTransform.position * (1 - alpha)\
        + s1.playerTransform.position * alpha
    newPlayerRotation := s0.playerTransform.rotation * (1 - alpha)\
        + s1.playerTransform.rotation * alpha
    newPlayerTransform := PlayerTransform {newPlayerPos, newPlayerRotation, s0.playerTransform.grounded}

    newCameraDist := s0.cameraData.distance * (1 - alpha)\
        + s1.cameraData.distance * alpha
    newCameraRotation := s0.cameraData.rotation * (1 - alpha)\
        + s1.cameraData.rotation * alpha
    newCameraData := PlayerFollowingCamera {newCameraDist, newCameraRotation}

    return GameState {newPlayerTransform, newCameraData}
}

get_camera_position :: proc(playerTransform: ^PlayerTransform, cameraData: PlayerFollowingCamera) -> rl.Vector3 {
    // direction projected to the XY plane.
    playerPosition := playerTransform.position
    playerFacingDirection := get_player_forward(playerTransform)
    rotatedDirection := rl.Vector2Rotate(playerFacingDirection, cameraData.rotation)
    cameraXZ := playerPosition.xz - (cameraData.distance * rotatedDirection)

    return rl.Vector3{cameraXZ.x, playerPosition.y + CAMERA_HEIGHT_DIFFERENCE, cameraXZ.y}
}

get_player_forward :: proc(transform: ^PlayerTransform) -> rl.Vector2 {
    start_direction := rl.Vector2 {1.0, 0.0}
    rotation_corrected := -transform.rotation * rl.DEG2RAD // Convert to radians and make the rotation clockwise

    return rl.Vector2Rotate(start_direction, rotation_corrected)
}

get_player_sideways :: proc(transform: ^PlayerTransform) -> rl.Vector2 {
    start_direction := rl.Vector2 {1.0, 0.0}
    rotation_corrected := -transform.rotation * rl.DEG2RAD // Convert to radians and make the rotation clockwise

    return rl.Vector2Rotate(start_direction, rotation_corrected + rl.PI/2)
}

get_movement_direction_from_input :: proc() -> rl.Vector2 {
    movement_direction := rl.Vector2(0)

    if (rl.IsKeyDown(rl.KeyboardKey.W)) {
        movement_direction.x += 1.0
    }
    if (rl.IsKeyDown(rl.KeyboardKey.S)) {
        movement_direction.x -= 1.0
    }
    if (rl.IsKeyDown(rl.KeyboardKey.A)) {
        movement_direction.y -= 1.0
    }
    if (rl.IsKeyDown(rl.KeyboardKey.D)) {
        movement_direction.y += 1.0
    }

    return movement_direction
}

GRAVITY :: 10.0
SPEED :: 5.0
SENSITIVITY :: 5.0

get_forward_input_force :: proc(playerTransform: ^PlayerTransform) -> Force
{
    movementFromInput := get_movement_direction_from_input()
    forwardVector := get_player_forward(playerTransform)
    forward3d := rl.Vector3 { forwardVector.x, 0, forwardVector.y }

    return Force { movementFromInput.x * SPEED * forward3d, true } 
}

get_sideways_input_force :: proc(playerTransform: ^PlayerTransform) -> Force
{
    movementFromInput := get_movement_direction_from_input()
    sidewaysVector := get_player_sideways(playerTransform)
    sideways3d := rl.Vector3 { sidewaysVector.x, 0, sidewaysVector.y }

    return Force { movementFromInput.y * SPEED *sideways3d, true } 
}

get_gravity_force :: proc() -> Force
{
    return Force {rl.Vector3{0, -GRAVITY, 0}, false}
} 

update_player_transform :: proc(playerTransform: ^PlayerTransform, dt: f32) {
    // Position
    movement_direction := get_movement_direction_from_input()

    player_forward := get_player_forward(playerTransform)
    player_right := rl.Vector2Rotate(player_forward, rl.PI/2.0)

    movement_direction_player_space := movement_direction.x * player_forward\
        + movement_direction.y * player_right

    movement_delta := movement_direction_player_space * SPEED * dt
    position_delta := rl.Vector3{movement_delta.x, 0.0, movement_delta.y}

    if !playerTransform.grounded {
        gravity_delta := rl.Vector3{0, -1, 0} * GRAVITY * dt
        position_delta += gravity_delta
    }

    playerTransform.position += position_delta

    // Rotation
    mouseDelta := rl.GetMouseDelta()
    if (abs(mouseDelta.x) > rl.EPSILON) {
        rotationAmount := -mouseDelta.x * SENSITIVITY * dt
        playerTransform.rotation += rotationAmount
        //rl.CameraYaw(&camera, rotationAmount, true) 
    }
}

get_initial_game_state :: proc() -> GameState
{
    playerTransform := PlayerTransform {position=PLAYER_INITIAL_POSITION, rotation=CAMERA_INITIAL_ROTATION, grounded=false}
    cameraData := PlayerFollowingCamera { distance=CAMERA_DISTANCE, rotation=CAMERA_INITIAL_ROTATION }

    return GameState {playerTransform, cameraData}
}


position_bounding_box :: proc(defaultBb: rl.BoundingBox, position: rl.Vector3, scale :f32= 1.0) -> rl.BoundingBox {
    return rl.BoundingBox { 
        scale * defaultBb.min + position, 
        scale * defaultBb.max + position }
}

get_bounding_box_collisions :: proc(box: rl.BoundingBox, colliders: []rl.BoundingBox) -> []rl.BoundingBox
{
    collided: [dynamic]rl.BoundingBox

    for collider in colliders {
        colliding := rl.CheckCollisionBoxes(box, collider)
        if colliding {
            append(&collided, collider)
        }
    }

    return collided[:]
}

// figure out the face of the bb which we collided with
// stop only the part of the vector which would take us into the collider, instead slide along the wall

TryGetCollidingBox :: proc(playerBb: rl.BoundingBox, colliders: []rl.BoundingBox) -> (rl.BoundingBox, bool)
{
    for collider in colliders {
        colliding := rl.CheckCollisionBoxes(playerBb, collider)
        if colliding {
            return collider, true
        }
    }

    return rl.BoundingBox{}, false
}

Interval :: struct {
    min: f32,
    max: f32,
}

IntervalOverlap :: proc(first, second: Interval) -> (overlap: f32, orientation: bool)
{
    firstIsToTheLeft := first.min < second.min

    separateLength := (first.max - first.min) + (second.max - second.min)
    overlappedLength := second.max - first.min if firstIsToTheLeft else first.max - second.min

    return separateLength - overlappedLength, firstIsToTheLeft
}

// When we collide at a corner, due to the discrete nature of the simulation
// we can overshoot and arrive at a situation where a different normal than we want is calculated.
// When this happens we do another check to see if the normal makes sense.
// This values is set to the maximum penetration amount that can happen in a single tick.
NORMAL_CONFIDENCE_TRESHOLD :: SPEED * DT

GetNormalOfCollidedFace :: proc(box1, box2: rl.BoundingBox, movementDirection: rl.Vector3) -> rl.Vector3
{
    xOverlap, xOrientation := IntervalOverlap(Interval{box1.min.x, box1.max.x}, Interval{box2.min.x, box2.max.x})
    yOverlap, yOrientation := IntervalOverlap(Interval{box1.min.y, box1.max.y}, Interval{box2.min.y, box2.max.y})
    zOverlap, zOrientation := IntervalOverlap(Interval{box1.min.z, box1.max.z}, Interval{box2.min.z, box2.max.z})

    xNormal := rl.Vector3{-1 if xOrientation else 1, 0, 0}
    yNormal := rl.Vector3{0, -1 if yOrientation else 1, 0}
    zNormal := rl.Vector3{0, 0, -1 if zOrientation else 1}

    if xOverlap < yOverlap && xOverlap < zOverlap 
    {
        secondSmallest := yOverlap if yOverlap < zOverlap else zOverlap
        penetrationDifference := secondSmallest - xOverlap
        if penetrationDifference < NORMAL_CONFIDENCE_TRESHOLD
        {
            otherNormal := yNormal if yOverlap < zOverlap else zNormal

            movementDirNormalized := rl.Vector3Normalize(movementDirection)
            xCos := rl.Vector3DotProduct(movementDirNormalized, xNormal)
            otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

            return xNormal if xCos < otherCos else otherNormal
        }

        return xNormal
    }

    if zOverlap < yOverlap && zOverlap < xOverlap
    {
        secondSmallest := yOverlap if yOverlap < xOverlap else xOverlap
        penetrationDifference := secondSmallest - zOverlap
        if penetrationDifference < NORMAL_CONFIDENCE_TRESHOLD
        {
            otherNormal := yNormal if yOverlap < xOverlap else xNormal

            movementDirNormalized := rl.Vector3Normalize(movementDirection)
            zCos := rl.Vector3DotProduct(movementDirNormalized, zNormal)
            otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

            return zNormal if zCos < otherCos else otherNormal
        }

        return zNormal
    }

    secondSmallest := zOverlap if zOverlap < xOverlap else xOverlap
    penetrationDifference := secondSmallest - yOverlap
    if penetrationDifference < NORMAL_CONFIDENCE_TRESHOLD
    {
        otherNormal := zNormal if zOverlap < xOverlap else xNormal

        movementDirNormalized := rl.Vector3Normalize(movementDirection)
        yCos := rl.Vector3DotProduct(movementDirNormalized, yNormal)
        otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

        return yNormal if yCos < otherCos else otherNormal
    }

    return yNormal
}

GetWallSlidingDirection :: proc(playerBox, collisioxBox: rl.BoundingBox, movementDirection: rl.Vector3) -> rl.Vector3
{
    collisionNormal := GetNormalOfCollidedFace(playerBox, collisioxBox, movementDirection)
    leftSlidingDir := collisionNormal.zyx
    rightSlidingDir := -collisionNormal.zyx

    movementDirNormalized := rl.Vector3Normalize(movementDirection)
    cosLeft := rl.Vector3DotProduct(movementDirNormalized, leftSlidingDir)
    cosRight := rl.Vector3DotProduct(movementDirNormalized, rightSlidingDir)
    if cosLeft - cosRight > rl.EPSILON
    {
        return leftSlidingDir
    }
    else if cosRight - cosLeft > rl.EPSILON
    {
        return rightSlidingDir
    }

    return movementDirection
}

Force :: struct {
    vector: rl.Vector3,
    canBeRedirected: bool,
}

ForceSource :: enum { InputForward, InputSideways, Gravity } 


main :: proc() {
    rl.InitWindow(800, 450, "raylib [core] example - basic window")

    rl.DisableCursor()
    rl.SetTargetFPS(FPS)


    initialState := get_initial_game_state()

    cameraInitialPosition := get_camera_position(&initialState.playerTransform, initialState.cameraData)

    //fmt.printfln("player pos: %f, %f, %f", playerPosition.x, playerPosition.y, playerPosition.z)
    fmt.printfln("camera pos: %f, %f, %f", cameraInitialPosition.x, cameraInitialPosition.y, cameraInitialPosition.z)
    
    // for row, rowInx in LEVEL {
    //     for col, colInx in row {
    //         if (col == u8(LevelSymbol.Spawn)) {
    //             cameraX := LEVEL_POSITION_X_START + f32(colInx) * TILE_SIZE
    //             cameraZ := LEVEL_POSITION_Z_START + f32(rowInx) * TILE_SIZE
    //             cameraInitialPosition = { cameraX, 2.0, cameraZ }
    //
    //             break;
    //         }
    //     }
    // }

    cameraMode := rl.CameraMode.THIRD_PERSON
    camera := rl.Camera3D { 
        cameraInitialPosition,
        initialState.playerTransform.position,
        rl.Vector3 {0.0, 1.0, 0.0},
        45.0,
        rl.CameraProjection.PERSPECTIVE }
    
    //cameraDir := rl.GetCameraForward(&camera)
    //fmt.printfln("camera dir: %f, %f, %f", cameraDir.x, cameraDir.y, cameraDir.z)

    lightingShader := rl.LoadShader("res/shaders/basic_lighting.vs", "res/shaders/basic_lighting.fs")
    
    image := rl.GenImageChecked(512, 512, 64, 64, rl.RED, rl.BLUE)
    texture := rl.LoadTextureFromImage(image)
    
    player := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, 1.0, 1.0))
    playerBb := rl.GetModelBoundingBox(player)
    // player is a Model
    // Model has a .transform matrix field
    // 
    // I want to 1. update player position on keyboard event
    // 2. also player rotation (yaw only)
    // this can all be encoded in the transform matrix, but when I want to draw the model, I only have functions that take position and or rotation as well. 
    
    // For the shader to work, I need to setup the texture sampler and use it in the fragment shader
    cube := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, 1.0, 1.0))
    cube.materials[0].shader = lightingShader
    cube.materials[0].maps[rl.MaterialMapIndex.ALBEDO].texture = texture

    colliders: [4]rl.BoundingBox
    colliders[0] = rl.BoundingBox { rl.Vector3{-5, 0, -5}, rl.Vector3 {5, 0, 5} }
    colliders[1] = position_bounding_box(playerBb, rl.Vector3{2.0, 2.0, 0.0})
    colliders[2] = position_bounding_box(playerBb, rl.Vector3{0.0, 4.0, 0.0})
    colliders[3] = position_bounding_box(playerBb, rl.Vector3{0.0, 2.0, 2.0})

    playerForces: [ForceSource]Force
    playerForces[.InputForward] = Force { rl.Vector3(0), true }
    playerForces[.InputSideways] = Force { rl.Vector3(0), true }
    playerForces[.Gravity] = Force { rl.Vector3(0), false }

    keysPickedUp := 0

    accumulator :f32= 0.0
    previousState := initialState
    currentState := initialState

    gameloop: for !rl.WindowShouldClose() {
        frameTime := rl.GetFrameTime()

        accumulator += frameTime
        for accumulator >= DT {
            previousState = currentState

            // --- this should be somewhere else so as not to pollute the update loop code
            // update forces based on input
            playerForces[.InputForward] = get_forward_input_force(&previousState.playerTransform)
            playerForces[.InputSideways] = get_sideways_input_force(&previousState.playerTransform)
            playerForces[.Gravity] = get_gravity_force()

            // foreach force, update transform and check collision
            for force in playerForces
            {
                forceBeforeState := currentState

                currentState.playerTransform.position += force.vector * DT
                xbb := position_bounding_box(playerBb, currentState.playerTransform.position, 2.0)
                collidedBox, collision := TryGetCollidingBox(xbb, colliders[:])
                if collision
                {
                    currentState = forceBeforeState

                    if force.canBeRedirected
                    {
                        redirected := GetWallSlidingDirection(xbb, collidedBox, force.vector)

                        currentState.playerTransform.position += redirected * SPEED * DT
                        playerBbAfterRedirect := position_bounding_box(playerBb, currentState.playerTransform.position, 2.0)

                        rBox, rCollision := TryGetCollidingBox(playerBbAfterRedirect, colliders[:])
                        if rCollision {
                            fmt.printfln("redirected: %f, %f, %f", redirected.x, redirected.y, redirected.z)
                            currentState = forceBeforeState
                        }
                    }
                }
            }

            // Rotation
            mouseDelta := rl.GetMouseDelta()
            if (abs(mouseDelta.x) > rl.EPSILON) {
                rotationAmount := -mouseDelta.x * SENSITIVITY * DT
                currentState.playerTransform.rotation += rotationAmount
                //rl.CameraYaw(&camera, rotationAmount, true) 
            }
            // --- this should be somewhere else so as not to pollute the update loop code

            accumulator -= DT
        }

        alpha := accumulator / DT
        renderState := interpolate_states(&previousState, &currentState, alpha)

        /// From this point on, renderState should be used instead of current/prev state

        camera.position = get_camera_position(&renderState.playerTransform, renderState.cameraData)
        camera.target = renderState.playerTransform.position

        rl.BeginDrawing()
            rl.ClearBackground(rl.RAYWHITE)
        
            rl.BeginMode3D(camera)

                rl.DrawPlane(rl.Vector3{ 0.0, 0.0, 0.0}, rl.Vector2{ 10, 10 }, rl.GREEN) // Draw ground

                // NOTE: this thing takes the rotation amount in degrees instead of radians -_-
                rl.DrawModelEx(player, renderState.playerTransform.position, rl.Vector3{0.0, 1.0, 0.0}, renderState.playerTransform.rotation, 2.0, rl.GOLD)
                rl.DrawModel(player, rl.Vector3{2.0, 2.0, 0.0}, 1.0, rl.RED)
                rl.DrawModel(player, rl.Vector3{0.0, 4.0, 0.0}, 1.0, rl.GREEN)
                rl.DrawModel(player, rl.Vector3{0.0, 2.0, 2.0}, 1.0, rl.BLUE)

                playerBbbw := position_bounding_box(playerBb, renderState.playerTransform.position, 2.0)
                rl.DrawBoundingBox(playerBbbw, rl.RED)

                for collider in colliders {
                    rl.DrawBoundingBox(collider, rl.RED)
                }
                //             rl.DrawModel(cube, wallPosition, wallScale, rl.BLUE)

                // for row, rowInx in varLevel {
                //     for col, colInx in row {
                //         positionX: f32 = LEVEL_POSITION_X_START + f32(colInx) * TILE_SIZE
                //         positionZ: f32 = LEVEL_POSITION_Z_START + f32(rowInx) * TILE_SIZE
                //
                //         switch col {
                //         case u8(LevelSymbol.Wall):
                //             wallScale: f32 = TILE_SIZE
                //             wallY := adjust_y_pos(0.0, wallScale, Pivot.bottom)
                //
                //             wallPosition := rl.Vector3{ positionX, wallY, positionZ }
                //
                //             rl.DrawModel(cube, wallPosition, wallScale, rl.BLUE)
                //         case u8(LevelSymbol.Key):
                //             keyScale: f32 = TILE_SIZE / 4.0
                //             keyY := adjust_y_pos(0.0, keyScale, Pivot.bottom)
                //             keyPosition := rl.Vector3{ positionX, keyY, positionZ }
                //
                //             rl.DrawModel(cube, keyPosition, keyScale, rl.GOLD)
                //         case u8(LevelSymbol.Goal):
                //             goalScale: f32 = TILE_SIZE / 2.0
                //             goalY := adjust_y_pos(0.0, goalScale, Pivot.bottom)
                //             goalPosition := rl.Vector3{ positionX, goalY, positionZ }
                //
                //             goalColor := keysPickedUp == KEYS_NEEDED ? rl.GREEN : rl.RED
                //             rl.DrawModel(cube, goalPosition, goalScale, goalColor)
                //         }
                //         
                //     }
                // }
                //
                //rl.DrawModel(cube, rl.Vector3{ 0.0, 1.5, 0.0 }, 3.0, rl.BLUE)
                //rl.DrawCube(rl.Vector3{ 0.0, 1.0, 0.0 }, 2.0, 2.0, 2.0, rl.BLUE)     // Draw a blue wall

            rl.EndMode3D()

            //debugMsg := fmt.tprintf("Keys: %i / %i", keysPickedUp, KEYS_NEEDED)
            //rl.DrawText(strings.clone_to_cstring(debugMsg), 10, 10, 20, rl.BLACK)
        rl.EndDrawing()
    }

    rl.UnloadModel(cube)
    rl.UnloadShader(lightingShader)
    rl.CloseWindow()
}
