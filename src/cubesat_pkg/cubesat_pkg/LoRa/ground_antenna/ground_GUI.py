import tkinter as tk
from tkinter import scrolledtext
from tkinter.ttk import LabelFrame, Label, Button, Progressbar
import time
from LoRa_ground import LoRaGround




class GroundGUI(tk.Tk):
    def __init__(self):
        self.root = tk.Tk()

        self.root.title("Antenne au sol - Interface")
        self.root.geometry("1000x500")
        self.root.configure(bg="#f5f5f5")
        self.create_widgets()


        self.lora = LoRaGround() 

        # create LoRa callbacks listeners
        self.lora.add_observer("gps_update", self.update_gps)
        self.lora.add_observer("new_message_received", lambda msg: self.add_history(msg, "received"))
        self.lora.add_observer("new_message_sent", lambda msg: self.add_history(msg, "sended"))
        self.lora.add_observer("new_log", self.add_log)
        self.lora.add_observer("file_transfert_percent", self.update_file_progressbar)
        self.lora.add_observer("file_transfert_end", self.end_transmission)


        
    def create_widgets(self):
        # frame for history and logs
        self.create_history_frame()

        # frame for gps infos
        self.create_gps_data_frame()

        # frame for inputs and buttons
        self.create_file_transfert_frame()

        # adjust column to make them resizable
        for c in range(3):
            self.root.columnconfigure(c, weight=1)
        
    def create_history_frame(self, height = 10):
        # Historique des messages
        frame = LabelFrame(self.root, text="Historiques", padding=10, height=height)
        frame.grid(row=0, column=0, columnspan=3, padx=10, pady=(10,0), sticky="we")

        # Titres au-dessus de chaque zone
        title_font = ("Arial", 12, "bold")
        Label(frame, text="Historique des messages :", font=title_font).grid(row=0, column=0)
        Label(frame, text="Logs de l'antenne au sol :",font=title_font).grid(row=0, column=1)

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
        Button(frame, text="Effacer historique", command=lambda: self.clear_history()).grid(row=3, column=0, padx=10)
        Button(frame, text="Effacer logs", command=lambda: self.clear_logs()).grid(row=3, column=1, padx=10)

        # Rendre les colonnes extensibles (avec le même poids)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)


    def create_gps_data_frame(self):
        # Zone d'affichage des données
        data_frame = LabelFrame(self.root, text="Données GPS", padding=10)
        data_frame.grid(row=1, column=0, padx=10, pady=(10,0))

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

    def create_file_transfert_frame(self):
        frame = LabelFrame(self.root, text="File transfert", padding=10)
        frame.grid(row=1, column=1, pady=10)

        # button
        self.picture_button = Button(frame)
        self.picture_button.grid(row=0, column=0)

        # progressbar
        self.transfer_progressbar = Progressbar(frame, orient="horizontal", length=100, mode="determinate")
        self.transfer_progressbar.grid(row=1, column=0, columnspan=2, pady=(10,0))

        # text
        self.file_transfert_txt = tk.StringVar()
        self.file_transfert_txt.set("No file transfert in progress")
        Label(frame, textvariable=self.file_transfert_txt).grid(row=0, column=1, sticky="w")

        # initialise the file transfert frame (names, values, ...)
        self.reset_file_transfert_frame()


    def add_history(self, msg, tag:str):
        if tag not in ["received", "sended"]: raise ValueError(f"Invalid message tag : '{tag}'\t must be 'received' or 'sended'.")

        self.messages_box['state'] = 'normal'
        self.messages_box.insert('end', str(msg) + '\n', tag)
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
    
    def clear_history(self):
        self.messages_box['state'] = 'normal'
        self.messages_box.delete('1.0', 'end')
        self.messages_box['state'] = 'disabled'
    
    def clear_logs(self):
        self.logs_box['state'] = 'normal'
        self.logs_box.delete('1.0', 'end')
        self.logs_box['state'] = 'disabled'


    def update_gps(self, gps_data):
        self.gps_status_txt.set(str(gps_data["status"]))
        self.lat_var.set(str(gps_data["latitude"]))
        self.long_var.set(str(gps_data["longitude"]))
        self.alt_var.set(str(gps_data["altitude"]))
        self.gps_last_update_txt.set(f"({round(time.time() - gps_data['last_update'])} s ago)")


    def reset_file_transfert_frame(self):
        self.transfer_progressbar['value'] = 0
        self.picture_button["text"] = "Ask for picture"
        self.picture_button["state"] = "enable"
        self.picture_button["command"] = self.picture_button_handler

    def picture_button_handler(self):
        self.lora.ask_for_picture()
        self.file_transfert_txt.set("Transfering file ...")
        self.picture_button["text"] = "Cancel tranfert"
        self.picture_button["command"] = lambda : self.lora.stop_file_transfert(); self.reset_file_transfert_frame()
    
    def update_file_progressbar(self, percent):
        self.transfer_progressbar['value'] = percent

    def end_transmission(self, message):
        if message == "success":
            self.file_transfert_txt.set("File received successfully")
        elif message == "abort":
            self.file_transfert_txt.set("File transfert abort")
        self.reset_file_transfert_frame()


if __name__ == "__main__":
    app = GroundGUI()
    app.root.mainloop()
