import airsim

client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
client.confirmConnection()

print("Connection successful.")
