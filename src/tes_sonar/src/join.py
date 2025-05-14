#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import math
import csv
import os
from datetime import datetime

# Menyimpan data kiri dan kanan
data_kiri = None
data_kanan = None

# Nama file CSV dan path lengkap
csv_file = os.path.expanduser("~/output_sonar.csv")

# Buat file dan tulis header jika belum ada
if not os.path.exists(csv_file):
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "distance_kiri_m", "distance_kanan_m", "kemiringan_deg"])

def callback_kiri(msg):
    global data_kiri
    data_kiri = msg.data
    tampilkan()

def callback_kanan(msg):
    global data_kanan
    data_kanan = msg.data
    tampilkan()

def tampilkan():
    if data_kiri is not None and data_kanan is not None:
        kemiringan = hitung_kemiringan(data_kiri, data_kanan)
        timestamp = datetime.now().isoformat()
        rospy.loginfo(f"[Kiri] {data_kiri} m | [Kanan] {data_kanan} m | Kemiringan: {kemiringan:.2f} derajat")

        # Tulis ke CSV
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, data_kiri, data_kanan, round(kemiringan, 2)])

def hitung_kemiringan(D_L, D_R, L=500):
    selisih_jarak = D_L - D_R
    theta_rad = math.atan(selisih_jarak / L)
    theta_deg = math.degrees(theta_rad)
    return theta_deg

def main():
    rospy.init_node('penggabung_node')
    rospy.Subscriber('/ping/left', Int32, callback_kiri)
    rospy.Subscriber('/ping/right', Int32, callback_kanan)
    rospy.loginfo("Node penggabung aktif... (data akan disimpan ke output_sonar.csv)")
    rospy.spin()

if __name__ == '__main__':
    main()
