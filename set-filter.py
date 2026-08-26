import json
import sys

if __name__ == "__main__":
    with open("calibrations/stereo_settings.json", "r") as file:
        data = json.load(file)
    data["filter"]["filter_on"] = int(sys.argv[1])
    with open("calibrations/stereo_settings.json", "w") as file:
        json.dump(data, file, indent=4)