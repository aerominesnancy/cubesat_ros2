import tkinter as tk
from tkinter import scrolledtext, messagebox
from tkinter.ttk import LabelFrame, Label, Frame, Button
import time
from cubesat_pkg.LoRa.ground_antenna.LoRa_ground import LoRaGround




class GroundGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Antenne au sol - Interface")
        self.geometry("1200x700")
        self.configure(bg="#f5f5f5")
        self.create_widgets_frames()

        self.lora = LoRaGround() 

        # create LoRa callbacks listeners
        self.lora.add_observer("gps_update", self.update_gps)
        self.lora.add_observer("new_message_received", lambda msg: self.add_history(msg, "received"))
        self.lora.add_observer("new_message_sent", lambda msg: self.add_history(msg, "sended"))
        self.lora.add_observer("new_log", self.add_log)
        

    def create_widgets_frames(self):
        self.history_frame()

        self.gps_position_frame()

        # Boutons d'action
        button_frame = Frame(self)
        button_frame.pack(pady=10)
        Button(button_frame, text="Demander une image", command=self.ask_image).grid(row=0, column=0, padx=10)
        Button(button_frame, text="GPS test", command=self.refresh_gps).grid(row=0, column=0, padx=10)
        

    

    def history_frame(self, height = 10):
        # Historique des messages
        frame = LabelFrame(self, text="Historiques", padding=10, height=height)
        frame.pack(fill="x", padx=10, pady=(10,0))

        # Titres au-dessus de chaque zone
        title_font = ("Arial", 12, "bold")
        Label(frame, text="Historique des messages :", font=title_font).grid(row=0, column=0)
        Label(frame, text="Logs de l'antenne au sol :",font=title_font).grid(row=0, column=1)

        # Zones ScrolledText côte à côte (C'EST DES OBJETS MUTABLE (BANGER pour les boutons))
        text_font =("Consolas", 10)
        self.messages_box = scrolledtext.ScrolledText(frame, height=height, state='disabled', font=text_font)
        self.messages_box.tag_config("received", foreground="green")    # change color of message with tag "received"
        self.messages_box.tag_config("sended", foreground="blue")       # change color of message with tag "sended"
        self.messages_box.grid(row=1, column=0)

        self.logs_box = scrolledtext.ScrolledText(frame, height=height, state='disabled', font=text_font)
        self.logs_box.tag_config("info", foreground="black")    # change color of message with tag "info"
        self.logs_box.tag_config("warn", foreground="orange")
        self.logs_box.tag_config("error", foreground="red")
        self.logs_box.tag_config("fatal", foreground="red", underline=True)
        self.logs_box.grid(row=1, column=1)

        # boutons
        Button(frame, text="Effacer historique", command=lambda: self.clear_history(self.messages_box)).grid(row=3, column=0, padx=10)
        Button(frame, text="Effacer logs", command=lambda: self.clear_history(self.logs_box)).grid(row=3, column=1, padx=10)

        # Rendre les colonnes extensibles (avec le même poids)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

    
    def gps_position_frame(self):
        # Zone d'affichage des données
        data_frame = LabelFrame(self, text="Données GPS", padding=10)
        data_frame.pack(padx=10, pady=(10,0))

        self.gps_status_txt = tk.StringVar(value="Not Fix")
        Label(data_frame, text="Status :").grid(row=0, column=0, sticky="w")
        Label(data_frame, textvariable=self.gps_status_txt).grid(row=0, column=1, sticky="w")

        self.gps_last_update_txt = tk.StringVar(value="(? s ago)") 
        Label(data_frame, textvariable=self.gps_last_update_txt).grid(row=0, column=3, sticky="w")

        self.lat_var = tk.StringVar(value="--")
        Label(data_frame, text="Latitude :").grid(row=1, column=0, sticky="w")
        Label(data_frame, textvariable=self.lat_var).grid(row=1, column=1, sticky="w")

        self.long_var = tk.StringVar(value="--")
        Label(data_frame, text="Longitude :").grid(row=2, column=0, sticky="w")
        Label(data_frame, textvariable=self.long_var).grid(row=2, column=1, sticky="w")
        
        self.alt_var = tk.StringVar(value="--")
        Label(data_frame, text="Altitude :").grid(row=3, column=0, sticky="w")
        Label(data_frame, textvariable=self.alt_var).grid(row=3, column=1, sticky="w")


    def add_history(self, msg, tag:str):
        if tag not in ["received", "sended"]: raise ValueError(f"Invalid message tag : '{tag}'\t must be 'received' or 'sended'.")

        self.messages_box['state'] = 'normal'
        self.messages_box.insert('end', msg + '\n', tag)
        self.messages_box['state'] = 'disabled'
        self.messages_box.see('end')

    def add_log(self, log_msg):
        self.logs_box['state'] = 'normal'
        if "[INFO]" in log_msg:
            self.logs_box.insert('end', log_msg + '\n', "info")
        elif "[ERROR]" in log_msg:
            self.logs_box.insert('end', log_msg + '\n', "error")
        elif "[WARN]" in log_msg:
            self.logs_box.insert('end', log_msg + '\n', "warn")
        elif "[FATAL]" in log_msg:
            self.logs_box.insert('end', log_msg + '\n', "fatal")
        else:
            self.logs_box.insert('end', log_msg + '\n')
        self.logs_box['state'] = 'disabled'
        self.logs_box.see('end')

    def ask_image(self):
        messagebox.showinfo("Action", "Demande de transfert d'image envoyée.")
        self.add_message("[Action] Demande de transfert d'image envoyée.")
        # TODO: Ajouter l'envoi réel de la commande à l'antenne

    def update_gps(self, gps_data):
        self.gps_status_txt.set(str(gps_data["status"]))
        self.lat_var.set(str(gps_data["latitude"]))
        self.long_var.set(str(gps_data["longitude"]))
        self.alt_var.set(str(gps_data["altitude"]))
        self.gps_last_update_txt.set(f"({round(time.time() - gps_data['last_update'])} s ago)")

    
    def clear_history(self, box):
        box['state'] = 'normal'
        box.delete('1.0', 'end')
        box['state'] = 'disabled'

        

if __name__ == "__main__":
    app = GroundGUI()
    app.mainloop()
