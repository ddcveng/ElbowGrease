package jamgame

import rl "vendor:raylib"
import "core:fmt"
import "core:strings"
import "core:math"
import "core:math/linalg"
import "core:slice"

FPS :: 60 // Frames per second
TPS :: 30 // Simulation tics per seconds
DT :: 1.0 / TPS // Delta time for the simulation

LEVEL_WIDTH :: 10
LEVEL_HEIGHT :: 10

TILE_SIZE :: 3.0

LEVEL_POSITION_X_START :: -LEVEL_WIDTH * TILE_SIZE / 2.0
LEVEL_POSITION_Z_START :: -LEVEL_HEIGHT * TILE_SIZE / 2.0

PLAYER_INITIAL_POSITION :: rl.Vector3{-43.0, 2.0, -3.0}
CAMERA_DISTANCE :: f32(6)
CAMERA_INITIAL_ROTATION :: f32(0)
CAMERA_HEIGHT_DIFFERENCE :: f32(1.5)

Point3 :: [3]f32
Triangle :: [3]Point3

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

// Consider adding "distinct" for more strict type checking.
// Right now I feel like its too annoying.
// - 4.3.2025
Rad :: f32
Deg :: f32

Player :: struct {
    position: rl.Vector3, // implicitly pivot.center
    rotation: Deg, // This is currenlty in degreees, since the rendering code takes it in degrees...
    // most of the calculations require this to be radians though so it kinda sucks

    jumpTimer: f32,
}

// Distance should only change when we would intersect a wall. IN this case zoom the camera in a little to avoid it.
// rotation will change when player rotates the camera independently from rotating the player character, when the player
// character rotates, this stays the same (the camera follows players rotation)
PlayerFollowingCamera :: struct {
    distance: f32,
    rotation: f32, // in radians, relative to players direction. 0 means the camera faces the same direction as the player. 
}

CollisionContext :: struct {
    playerCollider: rl.BoundingBox,

    sceneColliders: []rl.BoundingBox,
    sceneTriangleColliders: []Triangle
}

GameState :: struct {
    player: Player,
    //cameraData: PlayerFollowingCamera,
    cameraPitch: f32,
}

StaticData :: struct {
    playerModel: rl.Model,
    playerCollider: rl.BoundingBox,

    axisAlignedScene: rl.Model,
    angledScene: rl.Model,

    sceneAxisAlignedColliders: []rl.BoundingBox,
    sceneTriangleColliders: []Triangle,
}

setup_static_data :: proc() -> StaticData
{
    // @nocheckin having the player be a non-cube/ sphere will require changes to the collision detection logic
    playerModel := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, 1.0, 1.0))
    playerBoundingBox := rl.GetModelBoundingBox(playerModel)

    axisAlignedScene := rl.LoadModel("res/scenes/ikeamaze.glb")
    //axisAlignedScene.materials[1].shader = lightingShader
    // I can overwrite the shared on a per-mesh basis even though its just one model!
    // the color from the original material is lost though... maybe painting vertex colors would work?
    // what about textures from blender?? I guess I can acess them somehow if I find out where they are bound

    colliders := make([]rl.BoundingBox, axisAlignedScene.meshCount+1)
    for mesh, inx in axisAlignedScene.meshes[:axisAlignedScene.meshCount] {
        colliders[inx+1] = rl.GetMeshBoundingBox(mesh)
    }

    // ground collider
    colliders[0] = rl.BoundingBox { rl.Vector3{-100, 0, -100}, rl.Vector3 {100, 0, 100} }

    // collisions calculated by triangle intersection (these are rotated objects and stuff where the aabb is not good enough)
    angledScene := rl.LoadModel("res/scenes/sceneAngled.glb")
    angledMeshes := angledScene.meshes[:angledScene.meshCount]
    //triangleCount := slice.reduce(angledMeshes, 0, proc(accumulator: int, mesh: rl.Mesh) -> int { return accumulator + int(mesh.triangleCount)})
    //angledSceneTriangles := make([]Triangle, triangleCount)
    angledSceneTriangles: [dynamic]Triangle
    
    for mesh in angledMeshes {
        vertexComponents := mesh.vertices[:3 * mesh.vertexCount]
        vertices: [dynamic]Point3

        for i := 0; i < len(vertexComponents); i += 3 {
            vertex := Point3{ vertexComponents[i], vertexComponents[i+1], vertexComponents[i+2] }
            append(&vertices, vertex)
        }


        indices := mesh.indices[:3 * mesh.triangleCount]
        for i := 0; i < len(indices); i += 3 {
            triangle: Triangle = {vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]]}
            append(&angledSceneTriangles, triangle)
        }
    }

    return StaticData {playerModel, playerBoundingBox, axisAlignedScene, angledScene, colliders, angledSceneTriangles[:]}

}

// Hm.. keeping this up to date as the state grows will be tricky,
// can I maybe generate this? or say that if the actual leafs of  the state graph are vectors or scalars,
// I can automatically inspect the type and interpolate them ??
interpolate_states :: proc(s0: ^GameState, s1: ^GameState, alpha: f32) -> GameState {
    newPlayerPos := s0.player.position * (1 - alpha)\
        + s1.player.position * alpha
    newPlayerRotation := s0.player.rotation * (1 - alpha)\
        + s1.player.rotation * alpha
    newPlayer := Player {newPlayerPos, newPlayerRotation, s0.player.jumpTimer}

    newPitch := s0.cameraPitch * (1 - alpha) + s1.cameraPitch * alpha

    return GameState {newPlayer, newPitch}
}

get_camera_view_vector :: proc(state: GameState) -> rl.Vector3
{
    playerFacingDirection := get_player_forward(state.player)
    targetXZ := rl.Vector3 {playerFacingDirection.x, 0, playerFacingDirection.y}
    playerSideways := get_player_sideways(state.player)
    rotationAxis := rl.Vector3 {playerSideways.x, 0, playerSideways.y}

    viewVector := rl.Vector3RotateByAxisAngle(targetXZ, rotationAxis, state.cameraPitch * rl.DEG2RAD)
    return linalg.normalize(viewVector)
}

//first person camera
setup_camera :: proc(camera: ^rl.Camera, state: GameState)
{
    camera.position = state.player.position
    camera.target = camera.position + get_camera_view_vector(state)
}

// get_camera_position :: proc(player: Player, cameraData: PlayerFollowingCamera) -> rl.Vector3 {
//     // direction projected to the XY plane.
//     playerPosition := player.position
//     playerFacingDirection := get_player_forward(player)
//     rotatedDirection := rl.Vector2Rotate(playerFacingDirection, cameraData.rotation)
//     cameraXZ := playerPosition.xz - (cameraData.distance * rotatedDirection)
//
//     return rl.Vector3{cameraXZ.x, playerPosition.y + CAMERA_HEIGHT_DIFFERENCE, cameraXZ.y}
// }

get_player_forward :: proc(transform: Player) -> rl.Vector2 {
    start_direction := rl.Vector2 {1.0, 0.0}
    rotation_corrected := -transform.rotation * rl.DEG2RAD // Convert to radians and make the rotation clockwise

    return rl.Vector2Rotate(start_direction, rotation_corrected)
}

get_player_sideways :: proc(transform: Player) -> rl.Vector2 {
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

get_forward_input_force :: proc(player: Player) -> Force
{
    movementFromInput := get_movement_direction_from_input()
    forwardVector := get_player_forward(player)
    forward3d := rl.Vector3 { forwardVector.x, 0, forwardVector.y }

    return Force { movementFromInput.x * SPEED * forward3d, true } 
}

get_sideways_input_force :: proc(player: Player) -> Force
{
    movementFromInput := get_movement_direction_from_input()
    sidewaysVector := get_player_sideways(player)
    sideways3d := rl.Vector3 { sidewaysVector.x, 0, sidewaysVector.y }

    return Force { movementFromInput.y * SPEED *sideways3d, true } 
}

get_gravity_force :: proc() -> Force
{
    return Force {rl.Vector3{0, -GRAVITY, 0}, false}
} 

get_initial_game_state :: proc() -> GameState
{
    player := Player {position=PLAYER_INITIAL_POSITION, rotation=CAMERA_INITIAL_ROTATION, jumpTimer=0.0}
    return GameState {player, 0}
}


position_bounding_box :: proc(defaultBb: rl.BoundingBox, position: rl.Vector3, scale :f32= 1.0) -> rl.BoundingBox {
    return rl.BoundingBox { 
        scale * defaultBb.min + position, 
        scale * defaultBb.max + position }
}

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

PLAYER_COLLIDER_RADIUS :: f32(1.0)
TryGetCollidingTriangleNormal :: proc(playerPosition: Point3, triangles: []Triangle) -> (rl.Vector3, bool)
{
    for tri in triangles {
        closestPoint := closest_point_on_triangle(playerPosition, tri.x, tri.y, tri.z)
        distance := linalg.length(playerPosition - closestPoint)

        if distance < PLAYER_COLLIDER_RADIUS {
            u := tri.y - tri.x
            v := tri.z - tri.x
            normal := linalg.normalize(linalg.cross(u, v))

            return normal, true            
        }
    }

    return {}, false
}

CheckAnyCollision :: proc(player: Player, collisionContext: CollisionContext) -> bool
{
    playerCollider := position_bounding_box(collisionContext.playerCollider, player.position)

    box, collision := TryGetCollidingBox(playerCollider, collisionContext.sceneColliders)
    if collision {
        return true
    }

    //playerPosition := (playerBb.max - playerBb.min) / 2
    n, triCollision := TryGetCollidingTriangleNormal(player.position, collisionContext.sceneTriangleColliders)
    return triCollision
}
// CheckAnyCollision :: proc(playerBb: rl.BoundingBox, colliders: []rl.BoundingBox, triangles: []Triangle) -> bool
// {
//     box, collision := TryGetCollidingBox(playerBb, colliders)
//     if collision {
//         return true
//     }
//
//     playerPosition := (playerBb.max - playerBb.min) / 2
//     n, triCollision := TryGetCollidingTriangleNormal(playerPosition, triangles)
//     return triCollision
// }

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

MoveAndSlide :: proc(originalState: GameState, movementDirection: rl.Vector3, collisionContext: CollisionContext) -> GameState
{
    newState := originalState
    newState.player.position += movementDirection * SPEED * DT

    playerBbAfterMove := position_bounding_box(collisionContext.playerCollider, newState.player.position)
    collidedBox, collision := TryGetCollidingBox(playerBbAfterMove, collisionContext.sceneColliders)
    if collision
    {
        newState = originalState
        redirected := GetWallSlidingDirection(playerBbAfterMove, collidedBox, movementDirection)

        newState.player.position += redirected * SPEED * DT

        rCollision := CheckAnyCollision(newState.player, collisionContext)
        if rCollision {
            fmt.printfln("redirected: %f, %f, %f", redirected.x, redirected.y, redirected.z)
            return originalState
        }
    }

    // check collision with triangles 
    // find the normal of the triangle 
    triangleNormal, triCollision := TryGetCollidingTriangleNormal(newState.player.position, collisionContext.sceneTriangleColliders)
    if triCollision {
        newState = originalState

        directionDotNormal := linalg.dot(movementDirection, triangleNormal)
        if directionDotNormal < 0.0 {
            fmt.printfln("sliding along direction: %f, %f, %f", triangleNormal.x, triangleNormal.y, triangleNormal.z)
            projectedDirection := movementDirection - triangleNormal * directionDotNormal

            newState.player.position += projectedDirection * SPEED * DT

            slideCollision := CheckAnyCollision(newState.player, collisionContext)
            if slideCollision {
                fmt.printfln("slide failed in direction: %f, %f, %f", projectedDirection.x, projectedDirection.y, projectedDirection.z)
                return originalState
            }
        }
    }

    return newState
}

JUMP_HEIGHT :: 5.0
JUMP_DURATION_SECONDS :: 0.3
JUMP_SPEED :: JUMP_HEIGHT / JUMP_DURATION_SECONDS
ApplyVerticalMovement :: proc(originalState: GameState, actions: InputActions, collisionContext: CollisionContext) -> GameState
{
    // if any jump force remains, decay it and apply it
    //      if jump causes collision, stop the jump force at once
    jumpTimer := originalState.player.jumpTimer
    if jumpTimer > rl.EPSILON {
        jumpDt := min(jumpTimer, DT) 

        newState := originalState
        newState.player.position += rl.Vector3{0, 1, 0} * JUMP_SPEED * jumpDt

        collision := CheckAnyCollision(newState.player, collisionContext)
        if collision {
            collidedState := originalState
            collidedState.player.jumpTimer = 0

            return collidedState
        }

        newState.player.jumpTimer -= DT // I don't care the last iteration will make it negative
        return newState
    }
    
    // always to to apply gravity and if we collide with something, we know we are grounded.
    // if grounded, check for jump inputs
    gravityPositionDelta := rl.Vector3{0, -1, 0} * GRAVITY * DT
    newState := originalState
    newState.player.position += gravityPositionDelta

    collision := CheckAnyCollision(newState.player, collisionContext)
    if collision {
        groundedState := originalState

        if actions.jump {
            startJumpState := groundedState
            startJumpState.player.jumpTimer = JUMP_DURATION_SECONDS

            return startJumpState
        }

        return groundedState
    }

    return newState
}

handle_interaction :: proc(itemInteraction: ItemInteraction, itemManager: ^ItemManager)
{
    switch interaction in itemInteraction {
    case InteractableItem:
        pickup_item(itemManager, interaction.itemId)

    case PlaceableInCart:

    case PlaceableOnGround:

    case NoInteraction:
    }
}

INTERACTION_RADIUS :: f32(2.0 + PLAYER_COLLIDER_RADIUS)

get_possible_item_interaction :: proc(state: GameState, itemManager: ^ItemManager) -> ItemInteraction
{
    viewRay := rl.Ray{state.player.position, get_camera_view_vector(state)}

    if can_pickup_item(itemManager) {
        interactableItems := get_placed_items(itemManager)
        intersectedItem := ItemIdInvalid

        minIntersectionDistance := INTERACTION_RADIUS + rl.EPSILON
        for item in interactableItems {
            itemBox := itemManager.itemColliders[item.id]            
            collision := rl.GetRayCollisionBox(viewRay, itemBox)
            if collision.hit && collision.distance < minIntersectionDistance {
                minIntersectionDistance = collision.distance
                intersectedItem = item.id
            }
        }

        if intersectedItem != ItemIdInvalid {
            return InteractableItem{intersectedItem}
        }

        return NoInteraction{}
    }

    // the player is holding an item in hand
    // get the ray again
    // get intersection of the ray with scene geometry - that is the place where you want to place the item
    // check if the item can be placed there - bounding box check with correction?? TBD
    //      if not (common case), move the item along collided item normals to try and get it to a valid spot
    //      if the corrections cause a collision, its an invalid spot
    // if yes place

    // collision with cart should be easy enough.
    // if the ray intersects it, (but not through a wall!!), check the shopping list for the item
    // and return the corresponding result

    //TODO
    return NoInteraction{}
}

create_collision_context :: proc(staticData: ^StaticData, itemManager: ^ItemManager) -> CollisionContext
{
    placedItems := get_placed_items(itemManager)
    placedItemColliders := make([]rl.BoundingBox, len(placedItems))
    for item, i in placedItems {
        placedItemColliders[i] = itemManager.itemColliders[item.id]
    }

    colliderSlices := [][]rl.BoundingBox{ placedItemColliders, staticData.sceneAxisAlignedColliders }
    collidersMerged := slice.concatenate(colliderSlices)   

    return CollisionContext { staticData.playerCollider,  collidersMerged, staticData.sceneTriangleColliders}
}

FixedUpdate :: proc(previousState: GameState, actions: InputActions, staticData: ^StaticData, itemManager: ^ItemManager) -> GameState
{
    collisionContext := create_collision_context(staticData, itemManager)
    itemInteraction := get_possible_item_interaction(previousState, itemManager)

    currentState := ApplyVerticalMovement(previousState, actions, collisionContext)

    move := actions.movement
    if .Forward in move || .Backward in move {
        direction :f32= 1 if .Forward in move else -1
        forwardVector := get_player_forward(previousState.player)
        forward3d := rl.Vector3 { forwardVector.x, 0, forwardVector.y } * direction

        currentState = MoveAndSlide(currentState, forward3d, collisionContext)
    } 
    if .Left in move || .Right in move {
        direction :f32= 1 if .Right in move else -1
        sidewaysVector := get_player_sideways(previousState.player)
        sideways3d := rl.Vector3 { sidewaysVector.x, 0, sidewaysVector.y } * direction

        currentState = MoveAndSlide(currentState, sideways3d, collisionContext)
    }

    // Rotation
    if (abs(actions.cameraRotation.x) > rl.EPSILON) {
        rotationAmount := -actions.cameraRotation.x * SENSITIVITY * DT
        currentState.player.rotation += rotationAmount
    }
    if (abs(actions.cameraRotation.y) > rl.EPSILON) {
        rotationAmount := -actions.cameraRotation.y * SENSITIVITY * DT
        currentState.cameraPitch += rotationAmount
    }

    if actions.interact {
        handle_interaction(itemInteraction, itemManager)
    }

    return currentState
}


main :: proc() {
    rl.InitWindow(1600, 900, "raylib [core] example - basic window")

    rl.DisableCursor()
    rl.SetTargetFPS(FPS)

    staticData := setup_static_data()
    initialState := get_initial_game_state()

    itemManager := create_item_manager()
    create_item(&itemManager, PLAYER_INITIAL_POSITION + rl.Vector3{2.0, 0, 0}, { .Table, .Huge })

    cameraMode := rl.CameraMode.FIRST_PERSON
    camera := rl.Camera3D { 
        rl.Vector3(0),
        rl.Vector3(0),
        rl.Vector3 {0.0, 1.0, 0.0},
        45.0,
        rl.CameraProjection.PERSPECTIVE }

    setup_camera(&camera, initialState)
    
    //lightingShader := rl.LoadShader("res/shaders/basic_lighting.vs", "res/shaders/basic_lighting.fs")
    // collisions calculated by bounding box (AABB)
    // axisAlignedScene := rl.LoadModel("res/scenes/ikeamaze.glb")
    // //axisAlignedScene.materials[1].shader = lightingShader
    // // I can overwrite the shared on a per-mesh basis even though its just one model!
    // // the color from the original material is lost though... maybe painting vertex colors would work?
    // // what about textures from blender?? I guess I can acess them somehow if I find out where they are bound
    //
    // colliders := make([]rl.BoundingBox, axisAlignedScene.meshCount+1)
    // for mesh, inx in axisAlignedScene.meshes[:axisAlignedScene.meshCount] {
    //     colliders[inx+1] = rl.GetMeshBoundingBox(mesh)
    // }
    // colliders[0] = rl.BoundingBox { rl.Vector3{-100, 0, -100}, rl.Vector3 {100, 0, 100} }
    //
    // // collisions calculated by triangle intersection (these are rotated objects and stuff where the aabb is not good enough)
    // angledScene := rl.LoadModel("res/scenes/sceneAngled.glb")
    // angledMeshes := angledScene.meshes[:angledScene.meshCount]
    // //triangleCount := slice.reduce(angledMeshes, 0, proc(accumulator: int, mesh: rl.Mesh) -> int { return accumulator + int(mesh.triangleCount)})
    // //angledSceneTriangles := make([]Triangle, triangleCount)
    // angledSceneTriangles: [dynamic]Triangle
    // 
    // for mesh in angledMeshes {
    //     vertexComponents := mesh.vertices[:3 * mesh.vertexCount]
    //     vertices: [dynamic]Point3
    //
    //     for i := 0; i < len(vertexComponents); i += 3 {
    //         vertex := Point3{ vertexComponents[i], vertexComponents[i+1], vertexComponents[i+2] }
    //         append(&vertices, vertex)
    //     }
    //
    //
    //     indices := mesh.indices[:3 * mesh.triangleCount]
    //     for i := 0; i < len(indices); i += 3 {
    //         triangle: Triangle = {vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]]}
    //         append(&angledSceneTriangles, triangle)
    //     }
    // }

    
    //image := rl.GenImageChecked(512, 512, 64, 64, rl.RED, rl.BLUE)
    //texture := rl.LoadTextureFromImage(image)
    
    // playerModel := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, 1.0, 1.0))
    // playerBb := rl.GetModelBoundingBox(playerModel)
    // player is a Model
    // Model has a .transform matrix field
    // 
    // I want to 1. update player position on keyboard event
    // 2. also player rotation (yaw only)
    // this can all be encoded in the transform matrix, but when I want to draw the model, I only have functions that take position and or rotation as well. 
    
    // For the shader to work, I need to setup the texture sampler and use it in the fragment shader
    // cube := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, 1.0, 1.0))
    // cube.materials[0].shader = lightingShader
    // cube.materials[0].maps[rl.MaterialMapIndex.ALBEDO].texture = texture

    // colliders: [4]rl.BoundingBox
    // colliders[0] = rl.BoundingBox { rl.Vector3{-5, 0, -5}, rl.Vector3 {5, 0, 5} }
    // colliders[1] = position_bounding_box(playerBb, rl.Vector3{2.0, 2.0, 0.0})
    // colliders[2] = position_bounding_box(playerBb, rl.Vector3{0.0, 4.0, 0.0})
    // colliders[3] = position_bounding_box(playerBb, rl.Vector3{0.0, 2.0, 2.0})

    fixedUpdateRanLastFrame := false
    actions := InputActions{}

    accumulator :f32= 0.0
    previousState := initialState
    currentState := initialState

    gameloop: for !rl.WindowShouldClose() {
        frameTime := rl.GetFrameTime()

        actions = poll_actions_raw() if fixedUpdateRanLastFrame else poll_actions_inherit_queuable(actions)
        fixedUpdateRanLastFrame = false

        fmt.println("interact: ", actions.interact)
        accumulator += frameTime
        origAccumulator := accumulator
        for accumulator >= DT {
            previousState = currentState
            currentState = FixedUpdate(previousState, actions, &staticData, &itemManager)

            accumulator -= DT
            fixedUpdateRanLastFrame = true
        }

        alpha := accumulator / DT
        renderState := interpolate_states(&previousState, &currentState, alpha)

        /// From this point on, renderState should be used instead of current/prev state
        setup_camera(&camera, renderState)

        rl.BeginDrawing()
            rl.ClearBackground(rl.RAYWHITE)
        
            rl.BeginMode3D(camera)

                rl.DrawPlane(rl.Vector3{ 0.0, 0.0, 0.0}, rl.Vector2{ 200, 200 }, rl.GREEN) // Draw ground

                // NOTE: this thing takes the rotation amount in degrees instead of radians -_-
                //rl.DrawModelEx(player, renderState.player.position, rl.Vector3{0.0, 1.0, 0.0}, renderState.playerTransform.rotation, 2.0, rl.GOLD)

                rl.DrawModel(staticData.axisAlignedScene, rl.Vector3(0), 1.0, rl.WHITE)
                rl.DrawModel(staticData.angledScene, rl.Vector3(0), 1.0, rl.WHITE)

                //playerBbbw := position_bounding_box(playerBb, renderState.player.position, 2.0)
                //rl.DrawBoundingBox(playerBbbw, rl.RED)

                // for collider in colliders {
                //     rl.DrawBoundingBox(collider, rl.RED)
                // }

                for item in get_placed_items(&itemManager) {
                    rl.DrawModel(staticData.playerModel, item.position, 1.0, rl.GOLD) //TODO draw the right model...
                }

            rl.EndMode3D()

            rl.DrawFPS(50, 50)
            itemInHand := fmt.tprintf("in hand: %d", itemManager.activeItem)
            rl.DrawText(strings.clone_to_cstring(itemInHand), 50, 70, 20, rl.BLACK)

            //debugMsg := fmt.tprintf("Keys: %i / %i", keysPickedUp, KEYS_NEEDED)
            //rl.DrawText(strings.clone_to_cstring(debugMsg), 10, 10, 20, rl.BLACK)
        rl.EndDrawing()
    }

    // rl.UnloadModel(cube)
    // rl.UnloadShader(lightingShader)
    rl.CloseWindow()
}
