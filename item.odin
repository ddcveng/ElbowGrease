package jamgame

import rl "vendor:raylib"
import "core:container/small_array"
import "core:slice"

ItemType :: enum 
{
    Lamp,
    Chair,
    Table,
    Plant
}

ItemVariant :: enum 
{
    Red,
    Blue,
    Huge,
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

ITEMS_TO_BUY :: 8
ShoppingCart :: struct 
{
    position: Point3,
    connectedToPlayer: bool,

    shoppingList: [ITEMS_TO_BUY]ItemDescriptor,
    items: small_array.Small_Array(ITEMS_TO_BUY, ItemId)
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

    manager.activeItem = ItemIdInvalid

    for &item in manager.items {
        item.id = ItemIdInvalid
    }

    return manager
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

pickup_item :: proc(manager: ^ItemManager, id: ItemId)
{
    if manager.activeItem != ItemIdInvalid {
        panic("Cannot pickup 2 items at the same time")
    }

    if manager.items[id].id == ItemIdInvalid {
        panic("invalid item id")
    }

    manager.activeItem = id
}

place_item :: proc(manager: ^ItemManager, id: ItemId, position: Point3) 
{

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

// get interactable item
// cast a ray in the looking direction, if it intersects an item and the item is close enough, it is interactable
// items can either be picked up or placed. if an item is picked up, you cannot pick up another one. So in that case this logic never runs and we don't have to deal with it.
//      actually, I do need to handle this. I don't want to collide with the couch I am holding in my hand, so I need to disable that collider...
// when an item is placed, we need to update its position and bounding box placement.
