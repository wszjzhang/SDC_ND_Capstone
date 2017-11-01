from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf

from PIL import Image


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)

# label map and computation graph path
category_index = {1: {'id': 1, 'name': 'Green'}, 
                  2: {'id': 2, 'name': 'Red'}, 
                  3: {'id': 3, 'name': 'Yellow'}, 
                  4: {'id': 4, 'name': 'off'}}
                  
# download mode from https://files.slack.com/files-pri/T2HQV035L-F7TAUQF70/download/tlc_inference.zip
faster_rcnn_real_model = './frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self, model_graph_path=faster_rcnn_real_model):
        #TODO load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            
            # read serialized graph and load it into the detection graph
            with tf.gfile.GFile(model_graph_path, 'rb') as fid:    
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # Define input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            # class labels with the corresponding confidence level.
            self.scores_tensor = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.classes_tensor = self.detection_graph.get_tensor_by_name('detection_classes:0')
        
        # start tf session with the detection graph
        self.sess = tf.Session(graph=self.detection_graph)
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image_np = load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
              
        # Actual detection.
        (scores, classes) = self.sess.run(
              [self.scores_tensor, self.classes_tensor],
              feed_dict={self.image_tensor: image_np_expanded})

        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        return category_index[classes[np.argmax(scores)]]['name']
