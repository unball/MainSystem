#!/usr/bin/env python3

from view import View
from controller import Controller
from model import Model

def main():
  controller = Controller()
  view = View(controller)
  
  view.run()
  
  Model().flush()

if __name__ == "__main__":
  main()
