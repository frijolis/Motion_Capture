import pyrebase

def readDB(db):
    data = db.child("sensor_data").get()
    print(data.val())
    print("\n")

def updateDB(db, data):
    db.child("sensor_data").update(data)
    
def getlastDB(db):
    last = db.child("sensor_data").order_by_key().get()
    print(last.val())
    print("\n")

if __name__ == "__main__":
    #Setup
    config = { #This is for Firebase
        "apiKey": "AIzaSyD1ajof65FRErV16r4b1A8JRqliPdJllJU", 
        "authDomain": "cobey-2bbd1.firebaseapp.com",
        "databaseURL": "https://cobey-2bbd1.firebaseio.com",
        "storageBucket": "cobey-2bbd1.appspot.com"
    }
    firebase = pyrebase.initialize_app(config)
    db = firebase.database()
    readDB(db)
    data = {"timestamp1" :{"time":1, "sensor1":{"x":12, "y":323, "z":232}}}
    updateDB(db, data)
    readDB(db)
    getlastDB(db)
