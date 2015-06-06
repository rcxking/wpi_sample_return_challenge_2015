#!/usr/bin/python

'''
nn_node.py - Neural network node to predict if a sample is in the image or not.

RPI Rock Raiders
6/5/15

Last Updated: Bryant Pong: 6/5/15 - 10:39 PM   
'''

# Python Imports:
import rospy
import theano
import theano.tensor as T
import numpy as np
import cv2
import matplotlib.pyplot as plt
import lasagne 
import warnings
import os
import cPickle as pickle

# This function creates the neural network:
def build_network():

	print("nn_node: Now building neural network")
	l_in = lasagne.layers.InputLayer(
		shape=(None, 1, 112, 112))
	l_conv1 = lasagne.layers.Conv2DLayer(
		l_in,
		num_filters=48,
		filter_size=(5,5),
		stride=2,
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_pool1 = lasagne.layers.MaxPool2DLayer(
		l_conv1,
		pool_size=(3,3),
		stride=2)
	l_conv2 = lasagne.layers.Conv2DLayer(
		l_pool1,
		num_filters=64,
		filter_size=(5,5),
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_pool2 = lasagne.layers.MaxPool2DLayer(
		l_conv2,
		pool_size=(3,3),
		stride=2)
	l_conv3 = lasagne.layers.Conv2DLayer(
		l_pool2,
		num_filters=96,
		filter_size=(3,3),
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_conv4 = lasagne.layers.Conv2DLayer(
		l_conv3,
		num_filters=96,
		filter_size=(3,3),
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_conv5 = lasagne.layers.Conv2DLayer(
		l_conv4,
		num_filters=64,
		filter_size=(3,3),
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_pool3 = lasagne.layers.MaxPool2DLayer(
		l_conv5,
		pool_size=(3,3),
		stride=2)
	l_hidden1 = lasagne.layers.DenseLayer(
		l_pool3,
		num_units=256,
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_dropout1 = lasagne.layers.DropoutLayer(l_hidden1, p=0.5)
	l_hidden2 = lasagne.layers.DenseLayer(
		l_dropout1,
		num_units=256,
		nonlinearity=lasagne.nonlinearities.rectify,
		W=lasagne.init.HeNormal(gain='relu'))
	l_dropout2 = lasagne.layers.DropoutLayer(l_hidden2, p=0.5)
	l_output = lasagne.layers.DenseLayer(
		l_dropout2,
		num_units=2,
		nonlinearity=lasagne.nonlinearities.softmax)

	print("nn_node: Successfully created neural network")

	return l_output
'''
Main neural network node:
'''
def neural_network():

	# Disable Lasagne Warnings:
	warnings.filterwarnings('ignore', module='lasagne')
	
	# Create the neural network:
	output_layer = build_network()

if __name__ == "__main__":
	neural_network()
