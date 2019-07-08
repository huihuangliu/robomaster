
# -*- coding: utf-8 -*-


import numpy as np
import os
import signal, sys
from time import ctime, sleep
import tensorflow as tf
import random
#from PIL import Image
#import pylab
#import matplotlib.pyplot as plt


def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)


def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')


def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                        strides=[1, 2, 2, 1], padding='SAME')


def get_the_predict_num(img):
    max_val = y_conv.eval(feed_dict={x:img, keep_prob: 1.0})
    return np.argmax(max_val,1)


def check_idf(arr):
    temp = set(arr)
    if len(temp) == len(arr):
        return True  # 无相同元素
    else:
        return False # 有相同元素


# 创建卷积神经网络的输入，x为特征，y_为真实的label
x = tf.placeholder("float", shape=[None, 784])
y_ = tf.placeholder("float", [None, 10])

# 把x变成一个4d向量，-1：样本数量不定，28*28样本结构尺寸，1：颜色通道数
x_image = tf.reshape(x, [-1, 28, 28, 1])

# 第一层卷积，卷积核尺寸5*5,1个颜色通道，32个卷积核
W_conv1 = weight_variable([5, 5, 1, 32])
b_conv1 = bias_variable([32])

# 把x_image和权值向量进行卷积，加上偏置项，然后应用ReLU激活函数，
# 进行池化。
h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
h_pool1 = max_pool_2x2(h_conv1)

# 第二层卷积，卷积核尺寸：5*5,32个颜色通道，64个卷积核
W_conv2 = weight_variable([5, 5, 32, 64])
b_conv2 = bias_variable([64])

h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
h_pool2 = max_pool_2x2(h_conv2)

# 密集连接层，隐含节点为1024
# 经过两次池化边长变为7*7,第二个卷积核数量为64,故tensor尺寸为7*7*64
W_fc1 = weight_variable([7 * 7 * 64, 1024])
b_fc1 = bias_variable([1024])

# 将第二层卷积层输出转化为1D向量，-1：样本数量不定，并采用ReLU激活函数
h_pool2_flat = tf.reshape(h_pool2, [-1, 7*7*64])
h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

# 为了减少过拟合，在输出层之前加入dropout
keep_prob = tf.placeholder("float")
h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

# 添加一个softmax层，就像softmax regression一样
W_fc2 = weight_variable([1024, 10])
b_fc2 = bias_variable([10])
y_conv = tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

# 训练设置，采用交叉熵代价函数，输入为1列向量
cross_entropy = tf.reduce_mean(-tf.reduce_sum(y_*tf.log(y_conv),  reduction_indices=[1]))
# 采用Adam优化器，并给一个较小的学习速率
train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)

saver = tf.train.Saver()
# 测评准确率
correct_prediction = tf.equal(tf.argmax(y_conv, 1), tf.argmax(y_, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))

# 创建一个默认的会话，后面的操作便无需指定session了
sess_HW = tf.InteractiveSession()
saver.restore(sess_HW, '/home/allspark003/saver/HW_CNN')
sess_SMG = tf.Session()
saver.restore(sess_SMG, '/home/allspark003/saver/smg_model_final')
sess_Fire = tf.Session()
saver.restore(sess_Fire, '/home/allspark003/saver/fire_num')


def show_image(image):
    img = np.array(image)
    img = np.uint8(img.reshape(28, 28*9))
    img = Image.fromarray(img)
    plt.imshow(img, cmap='gray')
    plt.axis("off")
    pylab.show()


def run_fire(data, length):
    images_res = data
    new_image = [[], [], [], [], [], [], [], [], []]

    for i in range(len(images_res) / 28):
        j = i % 9
        new_image[j].extend(images_res[i * 28: (i + 1) * 28])
    images_p = []
    now_idf_data = []

    for num in range(9):
        images_p.append((np.matrix(new_image[num], dtype=np.float32)) / 255.0)
        box = sess_Fire.run(y_conv, feed_dict={x: images_p[num][0], keep_prob: 1.0})
        result = np.argmax(box,1)
        now_idf_data.append(int(result))

    #print "now_idf_data:",now_idf_data
    if check_idf(now_idf_data) == True:
        return tuple(now_idf_data)



def run_Hw(data, length):
    images_res = data
    new_image = [[], [], [], [], [], [], [], [], []]

    for i in range(len(images_res) / 28):
        j = i % 9
        new_image[j].extend(images_res[i * 28: (i + 1) * 28])
    images_p = []
    now_idf_data = []

    for num in range(9):
        images_p.append((np.matrix(new_image[num], dtype=np.float32)) / 255.0)
        box = sess_HW.run(y_conv, feed_dict={x: images_p[num][0], keep_prob: 1.0})
        result = np.argmax(box, 1)
        now_idf_data.append(int(result))

    if check_idf(now_idf_data) == True:
        #print "now_idf_data:",now_idf_data
        return tuple(now_idf_data)


def run_SMG(data, length):
    images_res = data
    new_image = [[], [], [], [], []]

    for i in range(len(images_res) / 28):
        j = i % 5
        new_image[j].extend(images_res[i * 28: (i + 1) * 28])
    images_p = []
    now_idf_data = []

    for num in range(5):
        images_p.append((np.matrix(new_image[num], dtype=np.float32)) / 255.0)
        box = sess_SMG.run(y_conv,feed_dict={x: images_p[num][0], keep_prob: 1.0})
        result = np.argmax(box,1)
        now_idf_data.append(int(result))

    if check_idf(now_idf_data) == True:
        #print "now_idf_data:",now_idf_data
        return tuple(now_idf_data)
