import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tensorflow as tf
from tensorflow.python.framework import ops

@ops.RegisterGradient("PlaceholderWithDefault")
def _IdGrad(_, grad):
    return grad

def load_graph(model_path):
    with tf.gfile.FastGFile(model_path, 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tf.import_graph_def(graph_def, name='')

def input_tensor_and_image_data(image):
    graph = tf.get_default_graph()
    if isinstance(image, str):
        return graph.get_tensor_by_name('DecodeJpeg/contents:0'), tf.gfile.FastGFile(image, 'rb').read()
    else:
        return graph.get_tensor_by_name('DecodeJpeg:0'), image

def decode_jpeg(image_path):
    # a bit overkill to use tensorflow for this, but we'll use its jpeg decoder to ensure consistency;
    # likely equivalent to matplotlib.image.imread
    graph = tf.get_default_graph()
    input_tensor = graph.get_tensor_by_name('DecodeJpeg/contents:0')
    raw_image_values_tensor = graph.get_tensor_by_name('DecodeJpeg:0')
    image_data = tf.gfile.FastGFile(image_path, 'rb').read()

    with tf.Session() as sess:
        return sess.run(raw_image_values_tensor, {input_tensor: image_data})

def classify_image(image, sess=None):
    graph = tf.get_default_graph()
    softmax_tensor = graph.get_tensor_by_name('final_result:0')
    input_tensor, image_data = input_tensor_and_image_data(image)
    
    if sess:
        class_probs = sess.run(softmax_tensor, {input_tensor: image_data})
    else:            
        with tf.Session() as sess:
            class_probs = sess.run(softmax_tensor, {input_tensor: image_data})
    class_probs = np.squeeze(class_probs)
    return class_probs