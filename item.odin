package jamgame

import rl "vendor:raylib"
import "core:slice"
import "core:fmt"
import "core:encoding/json"
import "core:os"

ItemType :: enum 
{
    Lamp,
    Chair,
    Table,
    Plant
}

ItemVariant :: enum 
{
    Regular,
    Red,
    Blue,
    Huge,
    Rare,
}

ItemDescriptor :: struct 
{
    type: ItemType, 
    variant: ItemVariant,
}

ItemId :: distinct int
ItemIdInvalid :: ItemId(-1)

Item :: struct 
{
    id: ItemId,
    position: Point3,
    descriptor: ItemDescriptor,
}

MAX_ITEMS :: 128

ItemManager :: struct 
{
    items: [MAX_ITEMS]Item,
    itemModels: [ItemType]rl.Model,
    itemColliders: [MAX_ITEMS]rl.BoundingBox,
    activeItem: ItemId, // the item you are currently holding

    itemIdCounter: int,
}

create_item_manager :: proc() -> ItemManager
{
    manager := ItemManager {}

    manager.itemModels[.Table] = rl.LoadModel("res/scenes/table.glb")
    manager.itemModels[.Lamp] = rl.LoadModel("res/scenes/lamp.glb")
    manager.itemModels[.Chair] = rl.LoadModel("res/scenes/chair.glb")
    manager.itemModels[.Plant] = rl.LoadModel("res/scenes/plant.glb")

    manager.activeItem = ItemIdInvalid

    for &item in manager.items {
        item.id = ItemIdInvalid
    }

    return manager
}

load_items_from_file :: proc(manager: ^ItemManager, filename: string)
{
    data, ok := os.read_entire_file_from_filename(filename)
    if !ok {
        fmt.eprintfln("Failed to load the file %s !", filename)
        return
    }
    defer delete(data)

    json_data, err := json.parse(data)
    if err != .None {
            fmt.eprintln("Failed to parse the json file.")
            fmt.eprintln("Error:", err)
            return
    }
    defer json.destroy_value(json_data)

    // Access the Root Level Object
    root := json_data.(json.Object)

    nodes := root["nodes"].(json.Array)

    itemPositions: [dynamic]Point3
    itemDescriptors: [dynamic]ItemDescriptor

    for node in nodes {
        nodeObj := node.(json.Object)
        extras := nodeObj["extras"].(json.Object)

        itemType, hasType := extras["ItemType"].(json.Float)
        if !hasType {
            itemType = f64(ItemType.Table)
        }

        itemVariant, hasVariant := extras["ItemVariant"].(json.Float)
        if !hasVariant {
            itemVariant = f64(ItemVariant.Regular)
        }

        descriptor := ItemDescriptor{ItemType(itemType), ItemVariant(itemVariant)}
        append(&itemDescriptors, descriptor)

        positionArr, hasPosition := nodeObj["translation"].(json.Array)
        position := Point3(0)
        if hasPosition {
            position.x = f32(positionArr[0].(json.Float))
            position.y = f32(positionArr[1].(json.Float))
            position.z = f32(positionArr[2].(json.Float))
        }
        append(&itemPositions, position)
    }

    for i in 0..<len(itemPositions) {
        create_item(manager, itemPositions[i], itemDescriptors[i])
    }
}

can_pickup_item :: proc(manager: ^ItemManager) -> bool
{
    return manager.activeItem == ItemIdInvalid
}

get_active_item :: proc(manager: ^ItemManager) -> Maybe(Item)
{
    if manager.activeItem == ItemIdInvalid {
        return nil
    } 

    return manager.items[manager.activeItem]
}

create_item :: proc(manager: ^ItemManager, itemPosition: Point3, itemDescriptor: ItemDescriptor) -> Item
{
    if manager.itemIdCounter >= MAX_ITEMS {
        panic("Maximum item amount reached!")
    }

    item := Item { ItemId(manager.itemIdCounter), itemPosition, itemDescriptor }
    manager.items[item.id] = item
    manager.itemColliders[item.id] = position_bounding_box(rl.GetModelBoundingBox(manager.itemModels[item.descriptor.type]), item.position)
    manager.itemIdCounter += 1

    return item
}

// items can be in 3 states: 
// placed,      collisions, can be interacted with to pick up
// picked up    no collisions, can be placed or put in the cart on interact (only 1 item picked up at a time)
// and in cart  no collitions, cannot be picked up - final state, can only be put here if the shopping list contains an item like this

pickup_item :: proc(manager: ^ItemManager, id: ItemId) -> Item
{
    if manager.activeItem != ItemIdInvalid {
        panic("Cannot pickup 2 items at the same time")
    }

    item := manager.items[id]
    if item.id == ItemIdInvalid {
        panic("invalid item id")
    }

    manager.activeItem = id
    return item
}

place_active_item :: proc(manager: ^ItemManager, position: Point3) 
{
    assert(manager.activeItem != ItemIdInvalid)

    manager.items[manager.activeItem].position = position
    manager.itemColliders[manager.activeItem] = position_bounding_box(manager.itemColliders[manager.activeItem], position)

    manager.activeItem = ItemIdInvalid
}

deposit_active_item_in_cart :: proc(manager: ^ItemManager, cart: ^ShoppingCart)
{
    assert(manager.activeItem != ItemIdInvalid)

    item := manager.items[manager.activeItem]
    add_item_to_cart(cart, item)

    // Disable the item and clear active item
    manager.items[manager.activeItem].id = ItemIdInvalid
    manager.activeItem = ItemIdInvalid
}

get_placed_items :: proc(manager: ^ItemManager) -> []Item
{
    allItems := slice.clone(manager.items[:])

    // Specially disable the currently held item
    if manager.activeItem != ItemIdInvalid {
        allItems[manager.activeItem].id = ItemIdInvalid
    }

    // Remove items that don't exist or are disabled, all others are placed
    placedItems := slice.filter(allItems, proc(item: Item) -> bool {
        return item.id != ItemIdInvalid
    })

    return placedItems
}

InteractableItem :: struct {
    itemId: ItemId,
}

PlaceableOnGround :: struct {
    spotValid: bool, // the player is aiming at a spot on the ground, but the spot may be invalid if the item cannot fit there
    spot: rl.Vector3,
}

PlaceableInCart :: struct {
    status: CartItemStatus,
}

NoInteraction :: struct {}

ItemInteraction :: union #no_nil {
    NoInteraction,
    InteractableItem,
    PlaceableOnGround,
    PlaceableInCart,
}

