import argparse
import numpy as np
import tensorflow as tf
from utils import *
import pdb

GRAPH = 'retrained_graph.pb'
with open('labels.txt', 'r') as f:
    LABELS = f.read().split()

def compute_brute_force_classification(image_path, nH=8, nW=8):
    raw_image = decode_jpeg(image_path)    # H x W x 3 numpy array (3 for each RGB color channel)

    window_predictions = np.zeros((nW,nH,2)) # Preallocated matrix for storing predictions

    # Create padded version of image
    padding = 15 # pixels

    # Extract dimensions of orig image and pad with zeros around edge
    height = raw_image.shape[0]
    width = raw_image.shape[1]
    rgb = raw_image.shape[2]

    paddedImage = raw_image.copy()
    npad = ((padding, padding), (padding, padding), (0,0)) # npad is a tuple of (n_before, n_after) for each dimension
    paddedImage = np.lib.pad(paddedImage,pad_width=npad, mode='constant', constant_values=0)

    # Coordinates for each window's upper left coordinate (after padding)
    segmentLength_x = width/nW
    segmentLength_y = height/nH
    xCoords = np.arange(nW)*segmentLength_x + padding
    yCoords = np.arange(nH)*segmentLength_y + padding

    # Cast to integers just in case points weren't evenly dividable
    xCoords = xCoords.astype(int)
    yCoords = yCoords.astype(int)
    
    # Loop over all windows
    for i in range(nW):
        for j in range(nH):
            x_cur = xCoords[i]
            y_cur = yCoords[j]

            startX = x_cur - padding
            startY = y_cur - padding
            endX = x_cur+segmentLength_x + padding
            endY = y_cur+segmentLength_y + padding

            with tf.Session() as sess:
                window = raw_image[startY:endY,startX:endX,:]
                window_predictions[i,j] = np.array(classify_image(window, sess))

    return window_predictions

def compute_convolutional_8x8_classification(image):
    graph = tf.get_default_graph()
    input_tensor, image_data = input_tensor_and_image_data(image)
    classification_input_tensor =  graph.get_tensor_by_name('input/BottleneckInputPlaceholder:0')
    classification_output_tensor = graph.get_tensor_by_name('final_result:0')
    convolution_ouput_tensor = graph.get_tensor_by_name('mixed_10/join:0')

    with tf.Session() as sess:
        convolution_output = sess.run(convolution_ouput_tensor, {input_tensor: image_data})
        predictions8x8 = sess.run(classification_output_tensor,
                                  {classification_input_tensor: np.reshape(convolution_output, [64,-1])})
        return np.reshape(predictions8x8, [8,8,2])

def compute_and_plot_saliency(image):
    graph = tf.get_default_graph()
    input_tensor, image_data = input_tensor_and_image_data(image)
    logits_tensor = graph.get_tensor_by_name('final_training_ops/Wx_plus_b/logits:0')
    imageHxWx3 = graph.get_tensor_by_name('Cast:0')

    with tf.Session() as sess:
        logits = np.squeeze(sess.run(logits_tensor, {input_tensor: image_data}))
        top_class = np.argmax(logits)
        gradient_tensor = tf.squeeze(tf.gradients(logits_tensor[0,top_class], imageHxWx3))
        raw_gradients = sess.run(gradient_tensor, {input_tensor: image_data})
    
    #### EDIT THIS CODE
    
    M = np.zeros(raw_gradients.shape[0:2])  # something of the right shape to plot

    for i in range(raw_gradients.shape[0]):
        for j in range(raw_gradients.shape[1]):
            M[i,j] = max(raw_gradients[i,j,:])

    ####

    plt.subplot(2,1,1)
    plt.imshow(M)
    plt.title('Saliency with respect to predicted class %s' % LABELS[top_class])
    plt.subplot(2,1,2)
    plt.imshow(decode_jpeg(image))
    plt.show()

def plot_classification(image_path, classification_array):
    nH, nW, _ = classification_array.shape
    image_data = decode_jpeg(image_path)
    aspect_ratio = float(image_data.shape[0]) / image_data.shape[1]
    plt.figure(figsize=(8, 8*aspect_ratio))
    p1 = plt.subplot(2,2,1)
    plt.imshow(classification_array[:,:,0], interpolation='none')
    plt.title('%s probability' % LABELS[0])
    p1.set_aspect(aspect_ratio*nW/nH)
    plt.colorbar()
    p2 = plt.subplot(2,2,2)
    plt.imshow(classification_array[:,:,1], interpolation='none')
    plt.title('%s probability' % LABELS[1])
    p2.set_aspect(aspect_ratio*nW/nH)
    plt.colorbar()
    plt.subplot(2,1,2)
    plt.imshow(image_data)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=str)
    parser.add_argument('--scheme', type=str)
    FLAGS, _ = parser.parse_known_args()

    load_graph(GRAPH)
    if FLAGS.scheme == 'brute':
        plot_classification(FLAGS.image, compute_brute_force_classification(FLAGS.image, 8, 8))
    elif FLAGS.scheme == 'conv':
        plot_classification(FLAGS.image, compute_convolutional_8x8_classification(FLAGS.image))
    elif FLAGS.scheme == 'saliency':
        compute_and_plot_saliency(FLAGS.image)
    else:
        print 'Unrecognized scheme:', FLAGS.scheme