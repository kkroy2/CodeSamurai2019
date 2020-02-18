from flask import Flask, render_template, url_for, session, redirect, request, send_from_directory
from bson import ObjectId
import logging
from logging.handlers import RotatingFileHandler
import traceback
import time

app = Flask(__name__)
app.secret_key = 'UIrfBrN*E(DNJ'


def getlats(i):
    ret = []
    file = open("Code Samurai 2019 - Problem Resources/Roadmap-Dhaka.csv").readlines()
    for i in range(len(file)):
        line = file[i]
        tokens = line.replace("\n", "").split(",")
        tokens = tokens[1:len(tokens) - 2]
        ret.append(tokens)
    return ret


@app.route('/')
def hello_world():
   path = getlats(0)[0:1000]
   return render_template("googlemap2.html", **locals())


if __name__ == '__main__':
   app.run()

