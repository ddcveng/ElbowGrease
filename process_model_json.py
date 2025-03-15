import json
import argparse

def process_json(input_file, output_file):
    try:
        # Open the original JSON file and load the data
        with open(input_file, 'r') as infile:
            data = json.load(infile)
        
        # Check if 'nodes' exists in the original JSON data
        if "nodes" not in data:
            raise KeyError("'nodes' key not found in the input file.")
        
        # Create the new structure for the 'nodes' key
        new_nodes = []

        for node in data["nodes"]:
            # Extract 'extras' and 'translation' from each node
            extras = node.get("extras", {})  # Default to an empty object if 'extras' doesn't exist
            translation = node.get("translation", [0.0, 0.0, 0.0])  # Default to [0.0, 0.0, 0.0] if 'translation' doesn't exist
            
            # Append a new object with only the 'extras' and 'translation' fields
            new_nodes.append({
                "extras": extras,
                "translation": translation
            })
        
        # Create the new JSON structure
        output_data = {
            "nodes": new_nodes
        }
        
        # Write the processed data to a new JSON file
        with open(output_file, 'w') as outfile:
            json.dump(output_data, outfile, indent=4)
        
        print(f"Processed JSON saved to {output_file}")
    
    except FileNotFoundError:
        print(f"Error: The file {input_file} was not found.")
    except json.JSONDecodeError:
        print("Error: Failed to decode JSON from the input file.")
    except KeyError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

# Function to parse command-line arguments
def main():
    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="Process a JSON file and extract 'extras' and 'translation' from each node.")
    parser.add_argument('input_file', type=str, help="Path to the input JSON file")
    parser.add_argument('output_file', type=str, help="Path to the output JSON file")

    # Parse arguments
    args = parser.parse_args()

    # Call the process_json function with the arguments
    process_json(args.input_file, args.output_file)

# Run the script
if __name__ == "__main__":
    main()
