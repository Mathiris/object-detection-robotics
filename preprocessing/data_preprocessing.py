import os
import cv2
import glob
import pandas as pd

WORKSPACE = os.getcwd() + "/"
DATA_DIR = WORKSPACE + "../datasets/"

STOPSIGN_TRAIN_LABEL = DATA_DIR + "stop_sign/train/Label/"
STOPSIGN_TRAIN_IMG = DATA_DIR + "stop_sign/train/"
STOPSIGN_TRAIN_IMG_GREYSCALE = STOPSIGN_TRAIN_IMG +"Stop_sign_greyscale/"
STOPSIGN_TRAIN_IMG_BLUR = STOPSIGN_TRAIN_IMG + "Stop_sign_blur/"
STOPSIGN_TRAIN_IMG_FLIP = STOPSIGN_TRAIN_IMG + "Stop_sign_flip/"

def rename_label(path_label):
    print("[!] - Processing renaming label...")
    
    for txt_file in glob.glob(path_label + "/*.txt"):
        f = open(txt_file, "r+")
        txt = f.read().replace("p s", "p_s")
        txt = txt.lower()
        f.seek(0)
        f.write(txt)
        f.truncate()
    f.close()

    print("[!] - Label renamed in all annotations files.")

def resize_bounding_box(path_label, path_image):
    print("\n[!] - Processing resizing BBox...")

    for element in os.listdir(path_label):
        list_content = []

        filename = str(element)
        filename = filename.replace("txt", "jpg")
        img = cv2.imread(path_image + "/" + filename, cv2.IMREAD_UNCHANGED)
        height = int(img.shape[0])
        width = int(img.shape[1])
        file = open(path_label + "/" + element, "r+")
        content = file.readlines()
        str_content = str(content[0])
        list_content = str_content.split()
                classe = list_content[0]
        hitbox_x_min = float(list_content[1])
        hitbox_x_max = float(list_content[2])
        hitbox_y_min = float(list_content[3])
        hitbox_y_max = float(list_content[4])
        hitbox_x_min = hitbox_x_min / width * 500
        hitbox_x_max = hitbox_x_max / width * 500
        hitbox_y_min = hitbox_y_min / height * 500
        hitbox_y_max = hitbox_y_max / height * 500
        string  = (classe + " " + str(hitbox_x_min) + " " + str(hitbox_x_max) + " " + str(hitbox_y_min) + " " + str(hitbox_y_max) + " ")
        file.seek(0)
        file.write(string)
        file.truncate()
        file.close

    print("[!] - Bounding boxes have been resized in all annotations files.")


def resize_image(path_image):
    print("\n[!] - Processing resizing images...")

    for element in os.listdir(path_image):
        if element.endswith(".jpg") or element.endswith(".jpeg") or element.endswith("png"):
            img = cv2.imread(path_image + "/" + element, cv2.IMREAD_UNCHANGED)
            height = int(img.shape[0])
            width = int(img.shape[1])
            dim = (height, width)
            resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
            cv2.imwrite(path_image + "/" + element, resized)
    
    print("[!] - All images have been resized!")
    
def data_aug (path_image, path_image_greyscale, path_image_blur, path_image_flip):
    for element in os.listdir(path_image):
        img = cv2.imread(path_image+'/'+element)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #element = element.replace('.jpg', '_greyscale.jpg')
        cv2.imwrite(path_image_greyscale+'/'+element,gray)
        img = cv2.imread(path_image+'/'+element)
        blur = cv2.GaussianBlur(img,(5,5),2)
        cv2.imwrite(path_image_blur+'/'+element,blur)
        img = cv2.imread(path_image+'/'+element)
        flipVertical = cv2.flip(img, 0)
        cv2.imwrite(path_image_flip+'/'+element,flipVertical)
        
def txt_to_csv(path_image, path_label):
    """Iterates through all .txt files (get from OIDV4_Toolkit) in a given directory and combines
    them in a single Pandas dataframe.

    Parameters:
    ----------
    path : str
        The path containing the .txt files
    Returns
    -------
    Pandas DataFrame
        The produced dataframe
    """

    txt_list = []

    for txt_file in glob.glob(path_label + "/*.txt"):
        f = open(txt_file, "r")
        txt = f.readlines()
        txt = str(txt[0]).split()
        txt_file = txt_file.split("\\")
        txt_file = txt_file[-1].replace("txt", "jpg")
        img = cv2.imread(path_image + "/" + txt_file, cv2.IMREAD_UNCHANGED)
        filename = txt_file
        label = txt[0] 
        width = int(img.shape[1])
        height = int(img.shape[0])
        x_min = float(txt[1])
        x_max = float(txt[2])
        y_min = float(txt[3])
        y_max = float(txt[4])

        for elem in txt:
            data = (filename,
                    label,
                    width,
                    height, 
                    x_min,
                    x_max,
                    y_min,
                    y_max)
            txt_list.append(data)

    column_name = ["filename", "label", "width", "height", "xmin", "ymin", "xmax", "ymax"]
    txt_df = pd.DataFrame(txt_list, columns=column_name)
    txt_df = txt_df.drop_duplicates()
    txt_df = txt_df.reset_index(drop=True)
    
    print(" > [+] - Create dataframe with all data annotations for each image.")

    return txt_df

rename_label(STOPSIGN_TRAIN_LABEL)
resize_bounding_box(STOPSIGN_TRAIN_LABEL, STOPSIGN_TRAIN_IMG)
resize_image(STOPSIGN_TRAIN_IMG)
data_aug(STOPSIGN_TRAIN_IMG_GREYSCALE, STOPSIGN_TRAIN_IMG_BLUR, STOPSIGN_TRAIN_IMG_FLIP)
stopSign_df = txt_to_csv(STOPSIGN_TRAIN_IMG, STOPSIGN_TRAIN_LABEL)


print(stopSign_df)
