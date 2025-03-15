package jamgame
import rl "vendor:raylib"
import "core:container/small_array"
import "core:slice"

ITEMS_TO_BUY :: 4

ShoppingCartStaticData :: struct 
{
    model: rl.Model,
    shoppingList: [ITEMS_TO_BUY]ItemDescriptor,
}

ShoppingCart :: struct 
{
    rigidBody: RigidBody,
    connectedToPlayer: bool,

    items: small_array.Small_Array(ITEMS_TO_BUY, ItemDescriptor)
}

add_item_to_cart :: proc(cart: ^ShoppingCart, item: Item)
{
    small_array.push_back(&cart.items, item.descriptor)
}

CartItemStatus :: enum {
    Acceptable,
    NotOnList,
    AlreadyInCart,
}

can_place_in_shopping_cart :: proc(cartStatic: ^ShoppingCartStaticData, cart: ShoppingCart, itemDescriptor: ItemDescriptor) -> CartItemStatus
{
    cartAcceptsItem := slice.contains(cartStatic.shoppingList[:], itemDescriptor)
    // @refactor; this is really sketchy??
    myItems := cart.items
    itemInCart := slice.contains(small_array.slice(&myItems), itemDescriptor)

    if cartAcceptsItem {
        return .AlreadyInCart if itemInCart else .Acceptable
    }

    return .NotOnList
}

