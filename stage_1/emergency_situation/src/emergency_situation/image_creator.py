#! /usr/bin/env python
# import Image from PIL http://www.pythonware.com/library/pil/handbook/introduction.htm
import Image


def ImageCreator(location_list, scale,origin, image_name, pkg_path, image_path):
    pixels = []
    moved_origin = []
    i = 0

    #for locations in location_list:
    moved_origin = ([location_list[0] + 0, location_list[1] + 0])
    pixels.append([moved_origin[0] / scale, moved_origin[1] / scale])
    print pixels

    #moved_origin = ([fire_location[0] + origin[0], fire_location[1] + origin[1]])
    #pixels_fire = [moved_origin[0] / scale, moved_origin[1] / scale]

    print "Opening images"
    #img_map = Image.open(image_path + image_name).convert("RGB")
    img_map = Image.open(pkg_path + "/config/subMap1.pgm").convert("RGB")
    img_cross = Image.open(pkg_path + "/config/cross.png")
    print "Resizing cross"
    resized_cross = img_cross.resize((15, 15))

    for pixel in pixels:
        print "Pasting the cross over the image (" + str(int(pixel[0])) + "," + str(int(pixel[1])) + ")"
        img_map.paste(resized_cross, (int(pixel[0]), int(pixel[1])), resized_cross)  # Pasting at pixel [10, 10]
        i = i + 1

    # img_map.save(pkg_path+"/config/map_with_cross.png")
    img_map.save(pkg_path+"/config/map_with_cross.png")

    print "Image is ready"
    return len(location_list)
