#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: sources
# Filename: model
# Created on: 2022/6/1


import tensorflow as tf
import matplotlib.pyplot as plt
import psutil
import time
import os
import random

LEARNING_RATE = 0.002


class customNN:
    def __init__(self, model_name="DNN"):
        self.name = model_name
        self.train_db = None
        self.test_db = None
        self.model = None
        self.train_size = 0
        self.test_size = 0
        self.data_shape = []
        self.batch_size = 8
        self.train_history = None
        self.tensorboard_enable = False
        self.log_root = "./tensorflow_log"
        self.callbacks = []
        self.callback_file_writer = None
        self.base_model = None
        self.epoch = 0
        self.model_file = "./{}.h5".format(self.name)
        self.autosave = False

    def _get_freeRAM(self):
        free_ram = psutil.virtual_memory().free
        return free_ram

    def _init_tensorboard(self):
        log_dir = os.path.join(self.log_root,
                               time.strftime("%Y%m%d-%H:%M:%S_") +
                               self.name
                               )
        tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir,
                                                              histogram_freq=1)
        self.callbacks.append(tensorboard_callback)
        self.callback_file_writer = tf.summary.create_file_writer(os.path.join(
            log_dir, "train"))
        self.callback_file_writer.set_as_default()

    def load_dataset(self, trainset, testset=None,
                     mapFunc=None, testRate=0.15, batchSize=8,
                     shufflePercentage=0.3, mapFuncTest=None):  # dataset has to be formated tensors: (data, labels)
        self.batch_size = batchSize
        if testset is None:
            # randomly split trainset and testset
            datasets = [ele for ele in trainset]
            try:
                data_dtype = datasets[0].dtype
                label_dtype = datasets[1].dtype
                dataDtype = data_dtype
                labelDtype = label_dtype
            except:
                pass
            train_size = len(datasets[0]) - int(len(datasets[0]) * testRate)
            all_indexs = list(range(len(datasets[0])))
            random.shuffle(all_indexs)
            trainset = [[], []]
            testset = [[], []]
            for index in all_indexs[:train_size]:
                data = datasets[0][index]
                label = datasets[1][index]
                trainset[0].append(data)
                trainset[1].append(label)
            for index in all_indexs[train_size:]:
                data = datasets[0][index]
                label = datasets[1][index]
                testset[0].append(data)
                testset[1].append(label)

        trainset = tuple(trainset)
        testset = tuple(testset)
        self.data_shape = tf.constant(trainset[0][0]).shape
        self.train_size = len(trainset[0])
        self.test_size = len(testset[0])
        if mapFunc is None:
            train_db = tf.data.Dataset.from_tensor_slices(trainset)
            test_db = tf.data.Dataset.from_tensor_slices(testset)
        else:
            if mapFuncTest is None:
                mapFuncTest = mapFunc
            self.data_shape = mapFunc(trainset[0][0]).shape
            train_db = tf.data.Dataset.from_tensor_slices(trainset[0])
            train_db = train_db.map(mapFunc, num_parallel_calls=tf.data.experimental.AUTOTUNE)
            train_db = tf.data.Dataset.zip((
                train_db, tf.data.Dataset.from_tensor_slices(trainset[1])))
            test_db = tf.data.Dataset.from_tensor_slices(testset[0])
            test_db = test_db.map(mapFuncTest)
            test_db = tf.data.Dataset.zip((
                test_db, tf.data.Dataset.from_tensor_slices(testset[1])))

        datasize = 1
        for size in self.data_shape:
            datasize *= size
        freeRAM = int(self._get_freeRAM() * shufflePercentage)
        shuffle_MaxbuffSize = int((freeRAM * 0.8) // datasize)
        prefetch_buffSize = int((freeRAM * 0.2) // (datasize * self.batch_size))

        shuffle_buffSize = shuffle_MaxbuffSize
        if shuffle_MaxbuffSize > self.train_size:
            shuffle_buffSize = self.train_size
        train_db = train_db.shuffle(shuffle_buffSize).repeat().batch(self.batch_size).prefetch(prefetch_buffSize)
        shuffle_buffSize = shuffle_MaxbuffSize
        if shuffle_MaxbuffSize > self.test_size:
            shuffle_buffSize = self.test_size
        test_db = test_db.shuffle(shuffle_buffSize).repeat().batch(self.batch_size).prefetch(prefetch_buffSize)

        self.train_db = train_db
        self.test_db = test_db

    def enable_tensorboard(self, log_dir_root="./tensorflow_log"):
        self.log_root = log_dir_root
        self.tensorboard_enable = True

    def add_callback(self, callback_func):  # all callbacks added will be reset after training
        self.callbacks.append(callback_func)

    def init_model(self):  # 神经网络模型
        model = tf.keras.Sequential()

        self.base_model = tf.keras.applications.Xception(weights="imagenet",
                                                         include_top=False,
                                                         input_shape=self.data_shape,
                                                         pooling="avg")  # 导入预训练网络卷积基
        self.base_model.trainable = False

        model.add(self.base_model)

        # model.add(tf.keras.layers.GlobalAveragePooling2D())

        model.add(tf.keras.layers.Dense(512, activation="relu"))
        model.add(tf.keras.layers.Dense(1, activation="sigmoid"))  # 2分类输出

        model.compile(optimizer=tf.optimizers.Adam(learning_rate=LEARNING_RATE),
                      loss="binary_crossentropy",  # 2分类问题
                      metrics=["acc"]
                      )

        self.model = model
        print(model.summary())

    def postProc_model(self):  # 模型后期处理（微调）
        model = self.model

        fine_tune_at = -33

        self.base_model.trainable = True

        for layer in self.base_model.layers[:fine_tune_at]:
            layer.trainable = False

        model.compile(optimizer=tf.optimizers.Adam(learning_rate=LEARNING_RATE / 10),
                      loss="binary_crossentropy",  # 2分类问题
                      metrics=["acc"]
                      )

        self.model = model
        print(model.summary())

    def save_model(self, path=None):
        if path is not None:
            self.model_file = path
        self.model.save(self.model_file)

    def load_model(self, path=None):
        if path is not None:
            self.model_file = path
        self.model = tf.keras.models.load_model(self.model_file)
        print(self.model.summary())

    def train(self, epochs=100):
        if self.tensorboard_enable and self.epoch == 0:
            self._init_tensorboard()
        try:
            self.train_history = self.model.fit(self.train_db,
                                                epochs=epochs,
                                                initial_epoch=self.epoch,
                                                steps_per_epoch=self.train_size // self.batch_size,
                                                validation_data=self.test_db,
                                                validation_steps=self.test_size // self.batch_size,
                                                callbacks=self.callbacks
                                                )
            self.epoch += epochs
        except KeyboardInterrupt:
            print("\ntraining process stopped manually")
            if self.autosave:
                self.load_model(self.model_file)

    def show_history_curves(self):
        plt.plot(self.train_history.epoch, self.train_history.history["loss"], label="Loss_Train")
        plt.plot(self.train_history.epoch, self.train_history.history["acc"], label="Acc_Train")
        plt.plot(self.train_history.epoch, self.train_history.history["val_loss"], label="Loss_Test")
        plt.plot(self.train_history.epoch, self.train_history.history["val_acc"], label="Acc_Test")
        plt.xlabel("Epoch")
        plt.ylabel("Value")
        plt.title(self.name)
        plt.legend()
        plt.show()

    def predict(self, data):
        if len(data.shape) != len(self.data_shape) + 1:
            data = tf.expand_dims(data, 0)
        res = self.model.predict(data)
        return res


def train():
    pass


if __name__ == '__main__':
    train()
