package jamgame
import rl "vendor:raylib"
import "core:container/small_array"

ITEMS_TO_BUY :: 1

ShoppingCartStaticData :: struct 
{
    model: rl.Model,
    shoppingList: [ITEMS_TO_BUY]ItemDescriptor,
}

ShoppingCart :: struct 
{
    rigidBody: RigidBody,
    connectedToPlayer: bool,

    items: small_array.Small_Array(ITEMS_TO_BUY, ItemId)
}


