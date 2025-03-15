package jamgame

import "core:fmt"
import "core:encoding/json"
import "core:os"

load_texture_indices :: proc(jsonFile: string) -> []TextureIndex
{
    data, ok := os.read_entire_file_from_filename(jsonFile)
    if !ok {
        fmt.eprintfln("Failed to load the file %s !", jsonFile)
        return {}
    }
    defer delete(data)

    json_data, err := json.parse(data)
    if err != .None {
            fmt.eprintln("Failed to parse the json file.")
            fmt.eprintln("Error:", err)
            return {}
    }
    defer json.destroy_value(json_data)

    // Access the Root Level Object
    root := json_data.(json.Object)

    nodes := root["nodes"].(json.Array)

    texIndices: [dynamic]TextureIndex
    for node in nodes {
        nodeObj := node.(json.Object)
        extras := nodeObj["extras"].(json.Object)

        texIndex, ok := extras["TextureIndex"].(json.Float)
        if !ok {
            // If no texture index, default to this
            texIndex = f64(TextureIndex.BlueMetal)
        }
        append(&texIndices, TextureIndex(texIndex))
    }

    return texIndices[:]

    // fmt.println("Root:")
    // fmt.println(
    //         "window_width:",
    //         root["window_width"],
    //         "window_height:",
    //         root["window_height"],
    //         "window_title:",
    //         root["window_title"],
    // )
    // fmt.println("rendering_api:", root["rendering_api"])
    //
    // // Store the value.
    // window_width := root["window_width"].(json.Float)
    // fmt.println("window_width:", window_width)
    //
    // fmt.println("")
    //
    // fmt.println("Renderer Settings:")
    // renderer_settings := root["renderer_settings"].(json.Object)
    // fmt.println("msaa:", renderer_settings["msaa"])
    // fmt.println("depth_testing:", renderer_settings["depth_testing"])
}
