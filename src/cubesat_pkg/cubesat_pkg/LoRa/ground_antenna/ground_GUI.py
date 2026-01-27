import tkinter as tk
from tkinter import scrolledtext, messagebox
from tkinter.ttk import LabelFrame, Label, Frame, Button
import time




class GroundGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Antenne au sol - Interface")
        self.geometry("1200x700")
        self.configure(bg="#f5f5f5")
        self.create_widgets_frames()
        

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
        Label(frame, text="Historique des messages reçus :", font=title_font).grid(row=0, column=0)
        Label(frame, text="Logs de l'antenne au sol :",      font=title_font).grid(row=0, column=1)

        # Zones ScrolledText côte à côte (C'EST DES OBJETS MUTABLE (BANGER pour les boutons))
        text_font =("Consolas", 10)
        self.messages_box = scrolledtext.ScrolledText(frame, height=height, state='disabled', font=text_font)
        self.messages_box.grid(row=1, column=0)
        self.logs_box = scrolledtext.ScrolledText(frame, height=height, state='disabled', font=text_font)
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


    def add_hitory(self, msg, box:str):
        if box == "log" :       box = self.log_box
        elif box == "history" : box = self.history_box
        else: raise ValueError("Invalid box name")

        box['state'] = 'normal'
        box.insert('end', msg + '\n')
        box['state'] = 'disabled'
        box.see('end')

    def ask_image(self):
        messagebox.showinfo("Action", "Demande de transfert d'image envoyée.")
        self.add_message("[Action] Demande de transfert d'image envoyée.")
        # TODO: Ajouter l'envoi réel de la commande à l'antenne

    def refresh_gps(self, status, latitude, longitude, altitude):
        # TODO: Récupérer les vraies données
        self.gps_status_txt.set(status)
        self.lat_var.set(latitude)
        self.long_var.set(longitude)
        self.alt_var.set(altitude)

    
    def clear_history(self, box):
        box['state'] = 'normal'
        box.delete('1.0', 'end')
        box['state'] = 'disabled'

        


class GUI_Logger():
    """ This logger is meant to write logs in the GUI log window."""
    def __init__(self, gui: GroundGUI): self.gui = gui
    def info(self, msg):  self.gui.add_hitory(f"{round(time.time(), 2)} [INFO]  {msg}", "log")
    def warn(self, msg):  self.gui.add_hitory(f"{round(time.time(), 2)} [WARN]  {msg}", "log")
    def error(self, msg): self.gui.add_hitory(f"{round(time.time(), 2)} [ERROR] {msg}", "log")
    def fatal(self, msg): self.gui.add_hitory(f"{round(time.time(), 2)} [FATAL] {msg}", "log")

    

if __name__ == "__main__":
    app = GroundGUI()
    app.mainloop()
