import pyrebase

def readDB(db):
    data = db.child("sensor_data").get()
    print(data.val())
    print("\n")

def updateDB(db, data):
    db.child("sensor_data").update(data)
    
def getlastDB(db):
    last = db.child("sensor_data").order_by_child("time").limit_to_last(1).get()
    print(last.val())
    print("\n")
    
def noquote(s):
	return s

if __name__ == "__main__":
    #Setup
    config = { #This is for Firebase
        "apiKey": "AIzaSyD1ajof65FRErV16r4b1A8JRqliPdJllJU", 
        "authDomain": "cobey-2bbd1.firebaseapp.com",
        "databaseURL": "https://cobey-2bbd1.firebaseio.com",
        "storageBucket": "cobey-2bbd1.appspot.com"
    }
    pyrebase.pyrebase.quote = noquote
    firebase = pyrebase.initialize_app(config)
    db = firebase.database()
    getlastDB(db)