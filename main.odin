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

PLAYER_HEIGHT :: 2.0
PLAYER_INITIAL_POSITION :: rl.Vector3{-41.0, 1.2, 1.0}
CAMERA_DISTANCE :: f32(6)
CAMERA_INITIAL_ROTATION :: f32(45)
CAMERA_HEIGHT_DIFFERENCE :: f32(1.5)

WINDOW_WIDTH :: 900
WINDOW_HEIGHT :: 720
SHEET_TILE_SIZE :: 1024
SHEET_RESIZED_TILE_SIZE :: f32(WINDOW_HEIGHT / 2)

HELD_ITEM_SIZE :: WINDOW_HEIGHT / 6.0

GRAVITY :: 10.0
SPEED :: 8.0
SENSITIVITY :: 5.0 // for mouse rotation
JUMP_FORCE :: 40.0

VELOCITY_DECAY_MULTIPLIER :: 0.9

Point3 :: [3]f32
Triangle :: [3]Point3

Pivot :: enum {
    center = 0,
    bottom = 1,
}

TextureIndex :: enum {
    Tiles,
    PorousStone,
    Carpet,
    BlueMetal,
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
    rigidBody: RigidBody,
    //position: rl.Vector3, // implicitly pivot.center
    rotation: Deg, // This is currenlty in degreees, since the rendering code takes it in degrees...
    // most of the calculations require this to be radians though so it kinda sucks
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
    cartCollider: rl.BoundingBox,

    sceneColliders: []rl.BoundingBox,
    itemColliders: []rl.BoundingBox,
    sceneTriangleColliders: []Triangle,
    ignoredItem: ItemId,
}

GameState :: struct {
    player: Player,
    //cameraData: PlayerFollowingCamera,
    cameraPitch: f32,
    interactAnimationTimer: f32,
    availableInteraction: ItemInteraction,
    heldItemGhostPosition: rl.Vector3, //only valid if held item
    shoppingCart: ShoppingCart,
    win: bool,
}

StaticData :: struct {
    playerModel: rl.Model,
    playerCollider: rl.BoundingBox,

    axisAlignedScene: rl.Model,
    //angledScene: rl.Model,

    sceneAxisAlignedColliders: []rl.BoundingBox,
    sceneTriangleColliders: []Triangle,

    shoppingCart: ShoppingCartStaticData,
    material_texture_atlas: rl.Texture2D,
    meshTextureIndices: []TextureIndex, // The texture index at index i of this array corresponds to the mesh at index i in the model.
}

setup_static_data :: proc() -> StaticData
{
    // @collision having the player be a non-cube/ sphere may require changes to the collision detection logic
    // probably boxes work fine, but the triangle radius might not be sufficient
    playerModel := rl.LoadModelFromMesh(rl.GenMeshCube(1.0, PLAYER_HEIGHT, 1.0))
    playerBoundingBox := rl.GetModelBoundingBox(playerModel)

    axisAlignedScene := rl.LoadModel("res/scenes/ikea2.glb")
    //axisAlignedScene.materials[1].shader = lightingShader
    // I can overwrite the shared on a per-mesh basis even though its just one model!
    // the color from the original material is lost though... maybe painting vertex colors would work?
    // what about textures from blender?? I guess I can acess them somehow if I find out where they are bound

    colliders := make([]rl.BoundingBox, axisAlignedScene.meshCount)
    for mesh, inx in axisAlignedScene.meshes[:axisAlignedScene.meshCount] {
        colliders[inx] = rl.GetMeshBoundingBox(mesh)
    }

    // ground collider
    //colliders[0] = rl.BoundingBox { rl.Vector3{-100, -10, -100}, rl.Vector3 {100, 0, 100} }

    // collisions calculated by triangle intersection (these are rotated objects and stuff where the aabb is not good enough)
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

    cartModel := rl.LoadModelFromMesh(rl.GenMeshCube(7.0, 5.0, 1.0))
    shoppingCartData := ShoppingCartStaticData { cartModel, { 
        ItemDescriptor{.Table, .Green},
        ItemDescriptor{.Chair, .Red},
        ItemDescriptor{.Plant, .Blue},
        ItemDescriptor{.Lamp, .Regular},
    } }

    material_textures := rl.LoadTexture("res/material_textures.png")
    materialIndices := load_texture_indices("res/level_material_indices.json")

    assert(len(materialIndices) == int(axisAlignedScene.meshCount))

    return StaticData {
        playerModel, playerBoundingBox, 
        axisAlignedScene, /*angledScene, */
        colliders, {},
        shoppingCartData,
        material_textures,
        materialIndices
    }
}

EmptyHand :: struct {
    spriteIndex: int,
}

HandHoldingItem :: struct {
    spriteIndex: int,
    itemTexture: rl.Texture2D,
}

HandState :: union #no_nil {
    EmptyHand,
    HandHoldingItem,
}

ANIMATION_LENGHT :: f32(0.4)
ANIMATED_ROTATION :: 30 // in degrees
HAND_X_OFFSET :: 200

draw_hand :: proc(handTex: rl.Texture2D, animationT: f32, handState: HandState)
{
    scale := f32(1)
    rotation := f32(0)

    if animationT > rl.EPSILON {
        progress := (ANIMATION_LENGHT - animationT) / ANIMATION_LENGHT 
        t := progress * rl.PI
        sinT := math.sin(t)

        rotation = ANIMATED_ROTATION * sinT
    }

    // NOTE: the code drawing the hand sprite is the same
    switch hand in handState {
    case EmptyHand: 
        // Bottom left corner of the texture
        origin := rl.Vector2{0, SHEET_RESIZED_TILE_SIZE}

        source := rl.Rectangle{SHEET_RESIZED_TILE_SIZE * f32(hand.spriteIndex), 0, SHEET_RESIZED_TILE_SIZE, SHEET_RESIZED_TILE_SIZE }

        position := rl.Vector2{origin.x + HAND_X_OFFSET, WINDOW_HEIGHT}
        dest := rl.Rectangle{position.x, position.y, SHEET_RESIZED_TILE_SIZE, SHEET_RESIZED_TILE_SIZE }

        rl.DrawTexturePro(handTex, source, dest, origin, rotation, rl.WHITE)
    case HandHoldingItem:
        // Bottom left corner of the texture
        origin := rl.Vector2{0, SHEET_RESIZED_TILE_SIZE}

        source := rl.Rectangle{SHEET_RESIZED_TILE_SIZE * f32(hand.spriteIndex), 0, SHEET_RESIZED_TILE_SIZE, SHEET_RESIZED_TILE_SIZE }

        position := rl.Vector2{origin.x + HAND_X_OFFSET, WINDOW_HEIGHT}
        dest := rl.Rectangle{position.x, position.y, SHEET_RESIZED_TILE_SIZE, SHEET_RESIZED_TILE_SIZE }

        rl.DrawTexturePro(handTex, source, dest, origin, rotation, rl.WHITE)
        
        // place the held item on the hand
        x := HAND_X_OFFSET + SHEET_RESIZED_TILE_SIZE / 2 - 70
        y := position.y - SHEET_RESIZED_TILE_SIZE + 20
        itemPosition := rl.Vector2{x, y}

        movedToOrigin := itemPosition - position
        rotated := rl.Vector2Rotate(movedToOrigin, rotation * rl.DEG2RAD)
        rotatedMovedBack := rotated + position

        rl.DrawTexture(hand.itemTexture, i32(rotatedMovedBack.x), i32(rotatedMovedBack.y), rl.WHITE)
    }
}

// Hm.. keeping this up to date as the state grows will be tricky,
// can I maybe generate this? or say that if the actual leafs of  the state graph are vectors or scalars,
// I can automatically inspect the type and interpolate them ??
interpolate_states :: proc(s0: ^GameState, s1: ^GameState, alpha: f32) -> GameState {
    newPlayerPos := s0.player.rigidBody.position * (1 - alpha)\
        + s1.player.rigidBody.position * alpha
    newPlayerRotation := s0.player.rotation * (1 - alpha)\
        + s1.player.rotation * alpha
    newPlayer := s0.player
    newPlayer.rigidBody.position = newPlayerPos
    newPlayer.rotation = newPlayerRotation

    newPitch := s0.cameraPitch * (1 - alpha) + s1.cameraPitch * alpha

    newIATimer:f32 = s0.interactAnimationTimer
    if s0.interactAnimationTimer > s1.interactAnimationTimer {
        newIATimer = s0.interactAnimationTimer * (1 - alpha) + s1.interactAnimationTimer * alpha
    }

    ghostPosition := s0.heldItemGhostPosition * (1 - alpha) + s1.heldItemGhostPosition * alpha

    cartPosition := s0.shoppingCart.rigidBody.position * (1 - alpha) + s1.shoppingCart.rigidBody.position * alpha
    newCart := s0.shoppingCart
    newCart.rigidBody.position = cartPosition

    return GameState {newPlayer, newPitch, newIATimer, s0.availableInteraction, ghostPosition, newCart, s0.win}
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

get_player_head_position :: proc(player: Player) -> rl.Vector3
{
    // anchor the camera at the top of the player bounding box
    return player.rigidBody.position //+ PLAYER_HEIGHT / 2.0
}

//first person camera
setup_camera :: proc(camera: ^rl.Camera, state: GameState)
{
    camera.position = get_player_head_position(state.player)
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

get_initial_game_state :: proc(staticData: ^StaticData) -> GameState
{
    playerBody := RigidBody {PLAYER_INITIAL_POSITION, rl.GetModelBoundingBox(staticData.playerModel), {}}
    player := Player {rigidBody=playerBody, rotation=CAMERA_INITIAL_ROTATION}

    cartBody := RigidBody {{46, 2.0, 39}, rl.GetModelBoundingBox(staticData.shoppingCart.model), {}}
    cart := ShoppingCart {cartBody, false, {}}

    return GameState {player, 0, 0.0, {}, {}, cart, false}
}

position_bounding_box :: proc(defaultBb: rl.BoundingBox, position: rl.Vector3) -> rl.BoundingBox {
    bbWithMinAtOrigin := rl.BoundingBox{
        rl.Vector3(0),
        defaultBb.max - defaultBb.min
    }

    centerPoint := bbWithMinAtOrigin.max / 2.0
    bbCenteredAroundOrigin := rl.BoundingBox {
        bbWithMinAtOrigin.min - centerPoint,
        bbWithMinAtOrigin.max - centerPoint
    }

    return rl.BoundingBox { 
        bbCenteredAroundOrigin.min + position, 
        bbCenteredAroundOrigin.max + position 
    }
}

get_rigid_body_bounding_box :: proc(body: RigidBody) -> rl.BoundingBox
{
    return position_bounding_box(body.boundingBox, body.position)
}

TryGetCollidingBox :: proc(playerBb: rl.BoundingBox, collisionContext: CollisionContext) -> (rl.BoundingBox, bool)
{
    for collider in collisionContext.sceneColliders {
        colliding := rl.CheckCollisionBoxes(playerBb, collider)
        if colliding {
            return collider, true
        }
    }

    for collider, i in collisionContext.itemColliders {
        if i == int(collisionContext.ignoredItem) {
            continue 
        }

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

check_any_collision :: proc(rigidBody: RigidBody, collisionContext: CollisionContext) -> bool
{
    bodyBox := position_bounding_box(rigidBody.boundingBox, rigidBody.position)
    box, collision := TryGetCollidingBox(bodyBox, collisionContext)
    if collision {
        return true
    }

    //playerPosition := (playerBb.max - playerBb.min) / 2
    n, triCollision := TryGetCollidingTriangleNormal(rigidBody.position, collisionContext.sceneTriangleColliders)
    return triCollision
}

// CheckAnyCollision :: proc(player: Player, collisionContext: CollisionContext) -> bool
// {
//     playerCollider := position_bounding_box(collisionContext.playerCollider, player.position)
//
//     box, collision := TryGetCollidingBox(playerCollider, collisionContext.sceneColliders)
//     if collision {
//         return true
//     }
//
//     //playerPosition := (playerBb.max - playerBb.min) / 2
//     n, triCollision := TryGetCollidingTriangleNormal(player.position, collisionContext.sceneTriangleColliders)
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

IntersectionDetails :: struct {
    collisionNormal: rl.Vector3,
    penetrationDepth: f32,
}

get_intersection_details :: proc(box1, box2: rl.BoundingBox, movementDirection: rl.Vector3) -> IntersectionDetails
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
            otherOverlap := min(yOverlap, zOverlap)

            movementDirNormalized := rl.Vector3Normalize(movementDirection)
            xCos := rl.Vector3DotProduct(movementDirNormalized, xNormal)
            otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

            return { xNormal, xOverlap } if xCos < otherCos else { otherNormal, otherOverlap }
        }

        return { xNormal, xOverlap }
    }

    if zOverlap < yOverlap && zOverlap < xOverlap
    {
        secondSmallest := yOverlap if yOverlap < xOverlap else xOverlap
        penetrationDifference := secondSmallest - zOverlap
        if penetrationDifference < NORMAL_CONFIDENCE_TRESHOLD
        {
            otherNormal := yNormal if yOverlap < xOverlap else xNormal
            otherOverlap := min(yOverlap, xOverlap)

            movementDirNormalized := rl.Vector3Normalize(movementDirection)
            zCos := rl.Vector3DotProduct(movementDirNormalized, zNormal)
            otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

            return {zNormal, zOverlap} if zCos < otherCos else {otherNormal, otherOverlap}
        }

        return {zNormal, zOverlap}
    }

    secondSmallest := zOverlap if zOverlap < xOverlap else xOverlap
    penetrationDifference := secondSmallest - yOverlap
    if penetrationDifference < NORMAL_CONFIDENCE_TRESHOLD
    {
        otherNormal := zNormal if zOverlap < xOverlap else xNormal
        otherOverlap := min(zOverlap, xOverlap)

        movementDirNormalized := rl.Vector3Normalize(movementDirection)
        yCos := rl.Vector3DotProduct(movementDirNormalized, yNormal)
        otherCos := rl.Vector3DotProduct(movementDirNormalized, otherNormal)

        return {yNormal, yOverlap} if yCos < otherCos else {otherNormal, otherOverlap}
    }

    return {yNormal, yOverlap}
}

GetNormalOfCollidedFace :: proc(box1, box2: rl.BoundingBox, movementDirection: rl.Vector3) -> rl.Vector3
{
    return get_intersection_details(box1, box2, movementDirection).collisionNormal
}

GetWallSlidingDirection :: proc(playerBox, collisioxBox: rl.BoundingBox, movementDirection: rl.Vector3) -> rl.Vector3
{
    collisionNormal := GetNormalOfCollidedFace(playerBox, collisioxBox, movementDirection)
    
    directionDotNormal := linalg.dot(movementDirection, collisionNormal)
    projectedDirection := movementDirection - collisionNormal * directionDotNormal

    return projectedDirection

    // leftSlidingDir := collisionNormal.zyx
    // rightSlidingDir := -collisionNormal.zyx
    //
    // movementDirNormalized := rl.Vector3Normalize(movementDirection)
    // cosLeft := rl.Vector3DotProduct(movementDirNormalized, leftSlidingDir)
    // cosRight := rl.Vector3DotProduct(movementDirNormalized, rightSlidingDir)
    // if cosLeft - cosRight > rl.EPSILON
    // {
    //     return leftSlidingDir
    // }
    // else if cosRight - cosLeft > rl.EPSILON
    // {
    //     return rightSlidingDir
    // }
    //
    // return movementDirection
}

Force :: struct {
    vector: rl.Vector3,
    canBeRedirected: bool,
}

ForceSource :: enum { InputForward, InputSideways, Gravity } 

// TODO; this is weird, I have to position the bb every time I want to use it...
RigidBody :: struct {
    position: Point3,
    boundingBox: rl.BoundingBox,
    velocity: rl.Vector3,
}

update_rigid_body :: proc(
    body: RigidBody,
    collisionContext: CollisionContext,
    instantVelocity: rl.Vector3 = rl.Vector3(0)) -> RigidBody
{
    newBody := body

    gravity := rl.Vector3 {0.0, -1.0, 0.0} * GRAVITY
    velocity := newBody.velocity + gravity + instantVelocity 

    verticalVelocity := rl.Vector3{0, velocity.y, 0}
    newBody.position += verticalVelocity * DT

    boxVertical := position_bounding_box(newBody.boundingBox, newBody.position)
    cBox, collision := TryGetCollidingBox(boxVertical, collisionContext)
    if collision {
        details := get_intersection_details(boxVertical, cBox, linalg.normalize(verticalVelocity))

        newBody.position += details.collisionNormal * details.penetrationDepth + rl.EPSILON
        newBody.velocity.y = 0
    }

    if abs(velocity.x) < rl.EPSILON && abs(velocity.y) < rl.EPSILON {
        return newBody
    }

    horizontalVelocity := rl.Vector3 {velocity.x, 0.0, velocity.z}
    horizontalBodyBefore := newBody
    horizontalBodyBefore.velocity = horizontalVelocity

    horizontalBodyAfter := move_and_slide(horizontalBodyBefore, collisionContext)

    newBody.position = horizontalBodyAfter.position
    newBody.velocity *= VELOCITY_DECAY_MULTIPLIER

    return newBody
}

move_and_slide :: proc(rigidBody: RigidBody, collisionContext: CollisionContext) -> RigidBody
{
    if abs(rigidBody.velocity.x) < rl.EPSILON \
    && abs(rigidBody.velocity.y) < rl.EPSILON \
    && abs(rigidBody.velocity.z) < rl.EPSILON {
        return rigidBody
    }

    body := rigidBody
    body.position += body.velocity * DT
    originalBodyDirection := linalg.normalize(rigidBody.velocity)
    originalSpeed := linalg.length(rigidBody.velocity)

    playerBbAfterMove := position_bounding_box(body.boundingBox, body.position)
    collidedBox, collision := TryGetCollidingBox(playerBbAfterMove, collisionContext)
    if collision
    {
        body = rigidBody
        redirected := GetWallSlidingDirection(playerBbAfterMove, collidedBox, originalBodyDirection)

        body.position += redirected * originalSpeed * DT

        rCollision := check_any_collision(body, collisionContext)
        if rCollision {
            //fmt.printfln("redirected: %f, %f, %f", redirected.x, redirected.y, redirected.z)
            return rigidBody
        }
    }

    // check collision with triangles 
    // find the normal of the triangle 
    triangleNormal, triCollision := TryGetCollidingTriangleNormal(body.position, collisionContext.sceneTriangleColliders)
    if triCollision {
        body = rigidBody

        directionDotNormal := linalg.dot(originalBodyDirection, triangleNormal)
        if directionDotNormal < 0.0 {
            //fmt.printfln("sliding along direction: %f, %f, %f", triangleNormal.x, triangleNormal.y, triangleNormal.z)
            projectedDirection := originalBodyDirection - triangleNormal * directionDotNormal

            body.position += projectedDirection * originalSpeed * DT

            slideCollision := check_any_collision(body, collisionContext)
            if slideCollision {
                //fmt.printfln("slide failed in direction: %f, %f, %f", projectedDirection.x, projectedDirection.y, projectedDirection.z)
                return rigidBody
            }
        }
    }

    return body
}

ray_x_scene :: proc (ray: rl.Ray, collisionContext: CollisionContext) -> rl.RayCollision
{
    minHitDistance := math.inf_f32(1)
    closestHit: rl.RayCollision

    for box in collisionContext.sceneColliders {
        collision := rl.GetRayCollisionBox(ray, box)
        if collision.hit && collision.distance < minHitDistance {
            minHitDistance = collision.distance
            closestHit = collision
        }
    }

    return closestHit    
}

handle_interaction :: proc(state: GameState, itemInteraction: ItemInteraction, itemManager: ^ItemManager) -> GameState
{
    stateAfterInteraction := state

    switch interaction in itemInteraction {
    case InteractableItem:
        item := pickup_item(itemManager, interaction.itemId)
        stateAfterInteraction.heldItemGhostPosition = item.rigidBody.position

    case PlaceableInCart:
        if interaction.status == .Acceptable {
            deposit_active_item_in_cart(itemManager, &stateAfterInteraction.shoppingCart)


            if stateAfterInteraction.shoppingCart.items.len == ITEMS_TO_BUY {
                stateAfterInteraction.win = true
            }
        }

    case PlaceableOnGround:
        if interaction.spotValid {
            place_active_item(itemManager, interaction.spot)
        }
    case Throw:
        item := get_active_item(itemManager).?
        itemManager.activeItem = ItemIdInvalid

        viewVector := get_camera_view_vector(state)
        newPosition := state.player.rigidBody.position + viewVector * 2.5

        itemManager.items[item.id].rigidBody.position = newPosition
        itemManager.items[item.id].rigidBody.velocity = viewVector * 30.0
    case NoInteraction:
    }

    return stateAfterInteraction
}

INTERACTION_RADIUS :: f32(5.0 + PLAYER_COLLIDER_RADIUS)

get_possible_item_interaction :: proc(
    state: GameState,
    itemManager: ^ItemManager,
    collisionContext: CollisionContext,
    staticData: ^StaticData) -> ItemInteraction
{
    viewRay := rl.Ray{get_player_head_position(state.player), get_camera_view_vector(state)}

    if can_pickup_item(itemManager) {
        interactableItems := get_placed_items(itemManager)
        intersectedItem := ItemIdInvalid

        minIntersectionDistance := INTERACTION_RADIUS + rl.EPSILON
        for item in interactableItems {
            itemBox := get_rigid_body_bounding_box(item.rigidBody)
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
    heldItem := get_active_item(itemManager).?
    itemBox := heldItem.rigidBody.boundingBox // not positioned yet

    // find the spot in the scene where we want to put it
    collision := ray_x_scene(viewRay, collisionContext)
    anySpot := collision.hit && collision.distance < INTERACTION_RADIUS + rl.EPSILON

    if !anySpot {
        return Throw{}
    }

    cartCollision := rl.GetRayCollisionBox(viewRay, collisionContext.cartCollider)
    placeInCart := cartCollision.hit && cartCollision.distance - collision.distance < rl.EPSILON
    if placeInCart {
        cartStatus := can_place_in_shopping_cart(&staticData.shoppingCart, state.shoppingCart, heldItem.descriptor)
        return PlaceableInCart{cartStatus}
    }

    spotToPlaceItem := collision.point

    itemBoxAtSpot := position_bounding_box(itemBox, spotToPlaceItem)
    ghostMovement := linalg.normalize(spotToPlaceItem - state.heldItemGhostPosition)

    normalsOfIntersectedFaces: [dynamic]IntersectionDetails
    sceneCollidersWithPlayer := slice.concatenate([][]rl.BoundingBox{collisionContext.sceneColliders, { collisionContext.playerCollider }})

    for box in sceneCollidersWithPlayer {
        collision := rl.CheckCollisionBoxes(itemBoxAtSpot, box)
        collision or_continue

        details := get_intersection_details(itemBoxAtSpot, box, ghostMovement)
        append(&normalsOfIntersectedFaces, details)
    }

    correctedPosition := spotToPlaceItem
    for collision in normalsOfIntersectedFaces {
        correctedPosition += collision.collisionNormal * (collision.penetrationDepth + rl.EPSILON)
    }

    boxAtCorrectedSpot := position_bounding_box(itemBox, correctedPosition)

    ok := true
    for box in sceneCollidersWithPlayer {
        ok = !rl.CheckCollisionBoxes(boxAtCorrectedSpot, box)
        ok or_break
    }

    return PlaceableOnGround{ok, correctedPosition}



    // get all colliding boxes (and triangles) with collision normals

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
    //return NoInteraction{}
}

create_collision_context :: proc(state: GameState, staticData: ^StaticData, itemManager: ^ItemManager) -> CollisionContext
{
    placedItems := get_placed_items(itemManager)
    placedItemColliders := make([]rl.BoundingBox, len(placedItems))
    for item, i in placedItems {
        //placedItemColliders[i] = itemManager.itemColliders[item.id]
        placedItemColliders[i] = get_rigid_body_bounding_box(item.rigidBody)
    }

    cartBoundingBox := get_rigid_body_bounding_box(state.shoppingCart.rigidBody)


    //colliderSlices := [][]rl.BoundingBox{ placedItemColliders, staticData.sceneAxisAlignedColliders, {cartBoundingBox} }
    //collidersMerged := slice.concatenate(colliderSlices)   

    return CollisionContext { 
        get_rigid_body_bounding_box(state.player.rigidBody), 
        cartBoundingBox, 
        staticData.sceneAxisAlignedColliders,
        placedItemColliders,
        staticData.sceneTriangleColliders,
        ItemIdInvalid}
}

FixedUpdate :: proc(previousState: GameState, actions: InputActions, staticData: ^StaticData, itemManager: ^ItemManager) -> GameState
{
    currentState := previousState

    collisionContext := create_collision_context(currentState, staticData, itemManager)
    availableInteraction := get_possible_item_interaction(currentState, itemManager, collisionContext, staticData)

    if pog, ok := availableInteraction.(PlaceableOnGround); ok {
        currentState.heldItemGhostPosition = pog.spot
    }

    move := actions.movement
    velocityFromActions: rl.Vector3

    if .Forward in move || .Backward in move {
        direction:f32 = 1 if .Forward in move else -1
        forwardVector := get_player_forward(previousState.player)
        forward3d := rl.Vector3 { forwardVector.x, 0, forwardVector.y } * direction

        velocityFromActions += forward3d
    } 
    if .Left in move || .Right in move {
        direction:f32 = 1 if .Right in move else -1
        sidewaysVector := get_player_sideways(previousState.player)
        sideways3d := rl.Vector3 { sidewaysVector.x, 0, sidewaysVector.y } * direction

        velocityFromActions += sideways3d
    }

    //velocityFromActions = linalg.normalize(velocityFromActions)
    velocityFromActions *= SPEED

    // for jumping find ray scene hit to ground. if its close enough say we are grounded.
    // if grounded and action.jump, add vertical force up to the body, not to the velocity from actions!
    rayToGround := rl.Ray{ currentState.player.rigidBody.position, rl.Vector3{0.0, -1.0, 0.0} }
    groundHit := ray_x_scene(rayToGround, collisionContext)
    playerGrounded := groundHit.hit && groundHit.distance < 1.0 + 0.001

    if playerGrounded && actions.jump {
        currentState.player.rigidBody.velocity.y += JUMP_FORCE
    }

    currentState.player.rigidBody = update_rigid_body(currentState.player.rigidBody, collisionContext, velocityFromActions)

    // Update shopping cart
    //currentState.shoppingCart.rigidBody = update_rigid_body(currentState.shoppingCart.rigidBody, collisionContext)

    // itemManager.items[0].rigidBody = update_rigid_body(itemManager.items[0].rigidBody, collisionContext)
    // fmt.println(itemManager.items[0].rigidBody.position)
    // fmt.println(itemManager.items[0].rigidBody.velocity)
    for i in 0..<len(itemManager.items)
    {
        itemActive := itemManager.items[i].id != ItemIdInvalid
        itemActive or_continue

        collisionContext.ignoredItem = ItemId(i)
        itemManager.items[i].rigidBody = update_rigid_body(itemManager.items[i].rigidBody,collisionContext)
        //fmt.println(item.rigidBody.position)
    }
    collisionContext.ignoredItem = ItemIdInvalid

    // currentState = ApplyVerticalMovement(currentState, actions, collisionContext)
    //
    // moveVector: rl.Vector3
    //
    // if .Forward in move || .Backward in move {
    //     direction:f32 = 1 if .Forward in move else -1
    //     forwardVector := get_player_forward(previousState.player)
    //     forward3d := rl.Vector3 { forwardVector.x, 0, forwardVector.y } * direction
    //
    //     moveVector += forward3d
    //     //currentState = MoveAndSlide(currentState, forward3d, collisionContext)
    // } 
    // if .Left in move || .Right in move {
    //     direction:f32 = 1 if .Right in move else -1
    //     sidewaysVector := get_player_sideways(previousState.player)
    //     sideways3d := rl.Vector3 { sidewaysVector.x, 0, sidewaysVector.y } * direction
    //
    //     moveVector += sideways3d
    //     //currentState = MoveAndSlide(currentState, sideways3d, collisionContext)
    // }
    //
    // currentState = MoveAndSlide(currentState, moveVector, collisionContext)

    // Rotation
    if (abs(actions.cameraRotation.x) > rl.EPSILON) {
        rotationAmount := -actions.cameraRotation.x * SENSITIVITY * DT
        //fmt.println(rotationAmount)
        currentState.player.rotation += rotationAmount
    }
    if (abs(actions.cameraRotation.y) > rl.EPSILON) {
        rotationAmount := -actions.cameraRotation.y * SENSITIVITY * DT
        currentState.cameraPitch += rotationAmount
    }

    interactOnCooldown := currentState.interactAnimationTimer > rl.EPSILON
    if interactOnCooldown {
        currentState.interactAnimationTimer -= DT
    }
    else {
        currentState.availableInteraction = availableInteraction

        if actions.interact && type_of(availableInteraction) != NoInteraction {
            currentState = handle_interaction(currentState, availableInteraction, itemManager)

            // put interact on cooldown
            currentState.interactAnimationTimer = ANIMATION_LENGHT 
            // the interaction was handled, clear it
            currentState.availableInteraction = NoInteraction{}
        }
    }

    return currentState
}

ITEM_ROTATION_SPEED :: 25.0
draw_held_item_to_texture :: proc(destinationTexture: ^rl.RenderTexture2D, itemModel: rl.Model, t: f64)
{
    itemPos := rl.Vector3{3, 0, 0}
    txCamera := rl.Camera3D { 
        rl.Vector3(0),
        itemPos,
        rl.Vector3 {0.0, 1.0, 0.0},
        45.0,
        rl.CameraProjection.PERSPECTIVE }

    itemBox := rl.GetModelBoundingBox(itemModel) // @speed; pass the bb to this method
    diagonalLength := linalg.length(itemBox.max - itemBox.min)
    scale := 2 / diagonalLength

    rl.BeginTextureMode(destinationTexture^)
    rl.ClearBackground(rl.BLANK)
    rl.BeginMode3D(txCamera)
        rl.DrawModelEx(itemModel, itemPos, rl.Vector3{0.0, 1.0, 1.0}, f32(t * ITEM_ROTATION_SPEED), rl.Vector3(scale), rl.WHITE)
    rl.EndMode3D()
    rl.EndTextureMode()
}

main :: proc() {
    rl.SetConfigFlags({rl.ConfigFlag.MSAA_4X_HINT, rl.ConfigFlag.WINDOW_RESIZABLE})
    rl.InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Elbow Grease")

    rl.DisableCursor()
    rl.SetTargetFPS(FPS)
    
    lightingShader := rl.LoadShader("res/shaders/basic_lighting.vs", "res/shaders/basic_lighting.fs")
    rednessLocation := rl.GetShaderLocation(lightingShader, strings.clone_to_cstring("textureIndex"))
    tilingLocation := rl.GetShaderLocation(lightingShader, strings.clone_to_cstring("meshDimensions"))
    variantLocation := rl.GetShaderLocation(lightingShader, strings.clone_to_cstring("variant"))
    texIndex: i32 = 0
    //rl.SetShaderValue(lightingShader, rednessLocation, &redness, rl.ShaderUniformDataType.FLOAT)


    // material2: rl.Material = { 
    //     shader = lightingShader,
    //     maps = make_multi_pointer([^]rl.MaterialMap, len(rl.MaterialMapIndex)) 
    // }
    // rl.SetMaterialTexture(&material2, rl.MaterialMapIndex.ALBEDO, texture)

    mesh := rl.GenMeshCube(1.0, 1.0, 1.0)
    // model := rl.LoadModelFromMesh(mesh)
    // model.materials[0].shader = lightingShader
    // model.materials[0].maps[rl.MaterialMapIndex.ALBEDO].texture = texture

    meshPos := PLAYER_INITIAL_POSITION + {6, 0, 6}
    meshTransform := rl.MatrixTranslate(meshPos.x, meshPos.y, meshPos.z)

    meshPos2 := PLAYER_INITIAL_POSITION + {8, 0, 6}
    meshTransform2 := rl.MatrixTranslate(meshPos2.x, meshPos2.y, meshPos2.z)

    staticData := setup_static_data()
    initialState := get_initial_game_state(&staticData)

    material: rl.Material = { 
        shader = lightingShader,
        maps = make_multi_pointer([^]rl.MaterialMap, len(rl.MaterialMapIndex)) 
    }
    rl.SetMaterialTexture(&material, rl.MaterialMapIndex.ALBEDO, staticData.material_texture_atlas)

    itemManager := create_item_manager()
    load_items_from_file(&itemManager, "res/items.json")
    // create_item(&itemManager, PLAYER_INITIAL_POSITION + rl.Vector3{2.0, 0.0, 0}, { .Lamp, .Huge })
    // create_item(&itemManager, PLAYER_INITIAL_POSITION + rl.Vector3{2.0, 0.0, 2}, { .Table, .Huge })
    // create_item(&itemManager, PLAYER_INITIAL_POSITION + rl.Vector3{2.0, 0.0, 4}, { .Plant, .Huge })
    // create_item(&itemManager, PLAYER_INITIAL_POSITION + rl.Vector3{4.0, 0.0, 4}, { .Chair, .Huge })
    //

    cameraMode := rl.CameraMode.FIRST_PERSON
    camera := rl.Camera3D { 
        rl.Vector3(0),
        rl.Vector3(0),
        rl.Vector3 {0.0, 1.0, 0.0},
        45.0,
        rl.CameraProjection.PERSPECTIVE }

    setup_camera(&camera, initialState)

    //fmt.println(camera.position)
    //fmt.println(camera.target)

    handImage := rl.LoadImage("res/hands_sheet.png")
    tiles := handImage.width / SHEET_TILE_SIZE

    imageAspectRatio := f32(handImage.width) / f32(handImage.height)
    rl.ImageResize(&handImage, i32(SHEET_RESIZED_TILE_SIZE * imageAspectRatio), i32(SHEET_RESIZED_TILE_SIZE))
    valid := rl.IsImageValid(handImage)

    handTex := rl.LoadTextureFromImage(handImage)
    heldItemTexture := rl.LoadRenderTexture(HELD_ITEM_SIZE, HELD_ITEM_SIZE)

    fixedUpdateRanLastFrame := false
    actions := InputActions{}

    accumulator :f32= 0.0
    previousState := initialState
    currentState := initialState

    gameloop: for !rl.WindowShouldClose() {
        frameTime := rl.GetFrameTime()

        if currentState.win {
            rl.BeginDrawing()
                rl.DrawText(strings.clone_to_cstring("You Win!"), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 60, rl.WHITE)
                rl.DrawText(strings.clone_to_cstring("press \"ESC\" to close the game"), WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2 + 60, 20, rl.WHITE)
            rl.EndDrawing()

            continue;
        }

        actions = poll_actions_raw() if fixedUpdateRanLastFrame else poll_actions_inherit_queuable(actions)
        fixedUpdateRanLastFrame = false

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
        //fmt.println(renderState.cameraPitch)
        // fmt.println(camera.target)

        rl.BeginDrawing()
            rl.ClearBackground(rl.RAYWHITE)
            rl.BeginMode3D(camera)

                tilingDefault := rl.Vector3(1.0)
                rl.SetShaderValue(lightingShader, tilingLocation, &tilingDefault, rl.ShaderUniformDataType.VEC3)
                // texIndex = 1
                // rl.SetShaderValue(lightingShader, rednessLocation, &texIndex, rl.ShaderUniformDataType.INT)
                // rl.DrawMesh(mesh, material, meshTransform)
                // 
                // texIndex = 0
                // rl.SetShaderValue(lightingShader, rednessLocation, &texIndex, rl.ShaderUniformDataType.INT)
                // rl.DrawMesh(mesh, material, meshTransform2)

                //rl.DrawMesh(mesh, material2, meshTransform2)

                // Draw the walls
                //rl.DrawModel(staticData.axisAlignedScene, rl.Vector3(0), 1.0, rl.WHITE)

                for mesh, inx in staticData.axisAlignedScene.meshes[:staticData.axisAlignedScene.meshCount] {
                    meshBb := staticData.sceneAxisAlignedColliders[inx]
                    tiling := (meshBb.max - meshBb.min)
                    rl.SetShaderValue(lightingShader, tilingLocation, &tiling, rl.ShaderUniformDataType.VEC3)
                    texIndex := staticData.meshTextureIndices[inx]
                    rl.SetShaderValue(lightingShader, rednessLocation, &texIndex, rl.ShaderUniformDataType.INT)


                    rl.DrawMesh(mesh, material, staticData.axisAlignedScene.transform)
                    //rl.DrawBoundingBox(meshBb, rl.WHITE)
                }
                //rl.DrawModel(staticData.angledScene, rl.Vector3(0), 1.0, rl.WHITE)

                texIndex = 1
                rl.SetShaderValue(lightingShader, rednessLocation, &texIndex, rl.ShaderUniformDataType.INT)
                // Draw items
                for item in get_placed_items(&itemManager) {
                    itemModel := itemManager.itemModels[item.descriptor.type]

                    variant := item.descriptor.variant
                    rl.SetShaderValue(lightingShader, variantLocation, &variant, rl.ShaderUniformDataType.INT)

                    pos := item.rigidBody.position
                    transform := rl.MatrixTranslate(pos.x, pos.y, pos.z)
                    for meshh in itemModel.meshes[:itemModel.meshCount] {
                        rl.DrawMesh(meshh, material, transform)
                    }
                    // //fmt.println(item.rigidBody.position)
                    // rl.DrawModel(itemManager.itemModels[item.descriptor.type], item.rigidBody.position, 1.0, rl.WHITE)
                    //rl.DrawBoundingBox(itemManager.itemColliders[item.id], rl.BLUE)
                }

                // Draw shopping cart
                rl.DrawModel(staticData.shoppingCart.model, renderState.shoppingCart.rigidBody.position, 1.0, rl.BROWN)

                // Draw ghost of placeable item
                if placeOnGround, ok := renderState.availableInteraction.(PlaceableOnGround); ok {
                    activeItem := get_active_item(&itemManager)
                    if item, itemOk := activeItem.?; itemOk {
                        itemBb := position_bounding_box(itemManager.items[item.id].rigidBody.boundingBox, renderState.heldItemGhostPosition)
                        color := rl.WHITE if placeOnGround.spotValid else rl.RED
                        rl.DrawBoundingBox(itemBb, color)
                    }
                }
            rl.EndMode3D()

            itemInHand := itemManager.activeItem != ItemIdInvalid
            if itemInHand {
                item := itemManager.items[itemManager.activeItem]
                draw_held_item_to_texture(&heldItemTexture, itemManager.itemModels[item.descriptor.type], rl.GetTime())
            }

            rl.DrawFPS(20, 20)

            // itemInHandMessage := fmt.tprintf("position: %x", renderState.player.rigidBody.position)
            // rl.DrawText(strings.clone_to_cstring(itemInHandMessage), 50, 70, 20, rl.BLACK)

            handState: HandState = HandHoldingItem{1, heldItemTexture.texture} if itemInHand else EmptyHand{0}
            draw_hand(handTex, renderState.interactAnimationTimer, handState)

            // Draw interaction help message
            message: string
            switch interaction in renderState.availableInteraction {
            case InteractableItem:
                message = "Press \"E\" to pickup the item"
            case PlaceableInCart:
                switch interaction.status {
                case .Acceptable:
                    message = "Press \"E\" to deposit item in cart"
                case .NotOnList:
                    message = "Item not on shopping list"
                case .AlreadyInCart:
                    message = "Item already in cart"
                }
            case PlaceableOnGround:
                message = "Press \"E\" to place the item on the ground" if interaction.spotValid else "The item cannot be placed here"
            case Throw:
                message = "Press \"E\" to throw the item"
            case NoInteraction:
            }

            if len(message) != 0 {
                messageCstring := strings.clone_to_cstring(message)
                textWidth := rl.MeasureText(messageCstring, 32)

                rl.DrawText(messageCstring, WINDOW_WIDTH / 2 - textWidth / 2, WINDOW_HEIGHT - 100, 32, rl.BLACK)
            }

            rl.DrawText(strings.clone_to_cstring("Shopping list:"), WINDOW_WIDTH - 175, 10, 25, rl.BLACK)

            for shoppingItem, i in staticData.shoppingCart.shoppingList {
                itemMsg := fmt.tprintf("%s %s", shoppingItem.variant, shoppingItem.type)
                x := i32(WINDOW_WIDTH - 175)
                y := i32(50 + i*30)
                rl.DrawText(strings.clone_to_cstring(itemMsg), x, y, 20, rl.BLACK)

                if can_place_in_shopping_cart(&staticData.shoppingCart, renderState.shoppingCart, shoppingItem) == CartItemStatus.AlreadyInCart {
                    rl.DrawLine(i32(x+10), i32(y+10), i32(x+10+100), i32(y+10), rl.RED)
                }
            }

            //debugMsg := fmt.tprintf("Keys: %i / %i", keysPickedUp, KEYS_NEEDED)
            //rl.DrawText(strings.clone_to_cstring(debugMsg), 10, 10, 20, rl.BLACK)
        rl.EndDrawing()
    }

    // rl.UnloadModel(cube)
    // rl.UnloadShader(lightingShader)
    rl.CloseWindow()
}
