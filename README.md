# Object detection - Robotics (Roomba)

## Setup and training

To train the models, we used this TensorFlow tutorial: https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/install.html
So it's required to follow the 'Install' part and the first section of 'Training Custom Object Detector'.

At the end you should have the same environment and folder tree as in the tutorial.

After downloading and labeling some images, we used Roboflow.com to format the whole dataset and fetch a TFRecord file that we put in the annotations folder, like in the tutorial.

We also downloaded a pre-trained model in the TensorFlow zoo here: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md
and put it in the correct directory.

Using the following command in the training folder, we launched the several trainings using a pipeline configured in a file in the same directory:
`python model_main_tf2.py --model_dir=models/ssd_mobilenet_v2_fpnlite_640x640 --pipeline_config_path=models/ssd_mobilenet_v2_fpnlite_640x640/pipeline.config`

During the training, we could monitor it with Tensorboard using:
`tensorboard --logdir=models/ssd_mobilenet_v2_fpnlite_640x640`

Finally to export the model and use it in the ROS env, we launched in the training folder:
`python .\exporter_main_v2.py --input_type image_tensor --pipeline_config_path .\models\ssd_mobilenet_v2_fpnlite_640x640\pipeline.config --trained_checkpoint_dir .\models\ssd_mobilenet_v2_fpnlite_640x640\ --output_directory .\exported-models\my_model`
