# Ros Vision Service: Rack products detection 

In production, the repo should contain the complete structure for a services who using the rack identifier, complete a trayectory for listing all the products with positional tracking and cloud database communication 

## Features, step by step 

- The service should read the DB and identify the rack section 
- Activate the scissors arm to move down over needes of each DB distance 
- Take pictures of both sides (turn around 180 degrees and turn back) 
- After complete all distances of the rack from DB, move to the top 
- Finally, send information to db if was complete detected all the products 

## Main Classes 

- Read NoSQL DB (Json) and get the vertical alignment in correspondance with the rack identifier 
- Send rostopics for scissors arm based on DB with intent to move up and down a certain distance 
- Camera client to take pictures and return frames on each required call 
- (Future feature) Camera should identify ArUco --no-longer-needed
- (Future feature) Autodetection for dirt on lenses 
- Rostopic to camera motor and rotate 180 degrees 

## Extra needed feature 

- Autocalibration of camera with lenses type eye fish 


## Fast Development Rules  

```python 
pip freeze > requirements.txt
``` 


