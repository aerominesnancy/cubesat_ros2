from LoRa_class import LoRa, encapsulate, Buffer
import time
import numpy as np
import struct

if __name__ == "__main__":

    buf = Buffer(LoRa.START_MARKER, LoRa.END_MARKER, LoRa.id_to_type)

    test_encapsulate = lambda msg, type: encapsulate(msg, type, LoRa.type_to_id, 
                                                     LoRa.paquet_size-LoRa.wrapper_size,
                                                     LoRa.START_MARKER, LoRa.END_MARKER)


    print("\n==================== Protocole de test LoRa_data_encapsulation.py ====================")


    print("\n\nTest 1 : Vérifie l'encapsulation correcte d'un message string et int.")
    # Test 1 : Encapsulation correcte d'un message string
    try:
        msg = "Hello, CubeSat!"
        print(f"\nTest 1 - Message de base : {msg} ('string')")
        encapsulated = test_encapsulate(msg, "string")
        print(f"Test 1 - Encapsulation obtenue : {encapsulated}")

        msg = 1234
        print(f"Test 1 - Message de base : {msg} ('int')")
        encapsulated = test_encapsulate(msg, "int")
        print(f"Test 1 - Encapsulation obtenue : {encapsulated}")
        
        msg = time.time()
        print(f"Test 1 - Message de base : {msg} ('timestamp_update')")
        encapsulated = test_encapsulate(msg, "timestamp_update")
        print(f"Test 1 - Encapsulation obtenue : {encapsulated}")

        print("✅ Test 1 OK: Encapsulation réussie.")
    except Exception as e:
        print(f"❌ Test 1 ECHEC: {e}")


    print("\n\nTest 2 : Vérifie que l'encapsulation d'un type non supporté lève une exception.")
    # Test 2 : Encapsulation d'un type non supporté
    try:
        msg = 12.34
        print(f"\nTest 2 - Valeur de base : {msg} ({type(msg)})")
        test_encapsulate(msg, "string")
        print("❌ Test 2 ECHEC: Exception non levée pour type non supporté.")
    except Exception as e:
        print(f"✅ Test 2 OK: Exception levée pour type non supporté. \nMessage : {e}")


    print("\n\nTest 3 : Vérifie l'extraction correcte d'un message encapsulé.")
    # Test 3 : Extraction correcte d'un message
    try:
        buf.clear()
        msg = "Hello, CubeSat!"
        encapsulated = test_encapsulate(msg, "string")
        buf.append(encapsulated)
        print("\nTest 3 - Message encapsulé ajouté au buffer: ", msg)
        print(f"Test 3 - Buffer après append : {buf.buffer}")
        result = buf.extract_message()
        print(f"Test 3 - Message extrait : {result}")
        assert result != None and result[1] == msg
        print("✅ Test 3.1 OK: Extraction correcte du message.")
    except AssertionError as e:
        print(f"❌ Test 3.1 ECHEC: Le message extrait ne correspond pas au message de base. {result}")
    
    try:
        print(f"Etat du buffer : {buf.buffer}")
        result = buf.extract_message()
        print(f"✅ Test 3.2 OK: Aucun message restant dans le buffer.")
    except Exception as e:
        print(f"❌ Test 3.2 ECHEC: {e}")        


    print("\n\nTest 4 : Vérifie la gestion d'un message incomplet au début du buffer (nettoyage et warning).")
    # Test 4 : Message incomplet au début du buffer
    try:
        buf.clear()
        encapsulated = test_encapsulate("test_4_incomplete", "string")
        buf.append(b'xxxx' + encapsulated)
        print(f"\nTest 4 - Buffer après append : {buf.buffer}")
        buf.extract_message()
        print("❌ Test 4 ECHEC: Warning non levé pour message incomplet.")
    except Warning as w:
        print(f"✅ Test 4.1 OK: Warning levé pour message incomplet. \nMessage : {w}")
    except Exception as e:
        print(f"❌ Test 4 ECHEC: Mauvaise exception levée: {e}")
    try:
        result = buf.extract_message()[1]
        print(f"Test 4 - Message extrait après nettoyage : {result}")
        assert result == "test_4_incomplete"
        print(f"✅ Test 4.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 4.2 OK: Le buffer n'a pas été nettoyé. \nMessage : {w}")


    print("\n\nTest 5 : Vérifie la détection d'un type de données inconnu dans le message.")
    # Test 5 : Type inconnu dans le message
    try:
        bad_type = b'\xAA\xBB\xFF\x00\x05hello\xCC\xDD'  # type 0xFF inconnu
        print(f"\nTest 5 - Message encapsulé avec type inconnu : {bad_type}")
        buf.clear()
        buf.append(bad_type)
        buf.extract_message()
        print("❌ Test 5.1 ECHEC: Exception non levée pour type inconnu.")
    except Exception as e:
        print(f"✅ Test 5.1 OK: Exception levée pour type inconnu. \nMessage : {e}")
    
    try:
        result = buf.extract_message()[1]
        print(f"Test 5 - Message extrait après nettoyage : {result}")
        assert result is None
        print(f"✅ Test 5.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 5.2 OK: Le buffer n'a pas été nettoyé. \nMessage : {w}")


    print("\n\nTest 6 : Vérifie la détection d'une longueur incorrecte dans le message.")
    # Test 6 : Longueur incorrecte dans le message
    try:
        # Longueur annoncée 10, mais seulement 5 octets de données
        bad_length = b'\xAA\xBB\x02\x00\x0Ahello\xCC\xDD'
        print(f"\nTest 6 - Message encapsulé avec mauvaise longueur : {bad_length}")
        buf.clear()
        buf.append(bad_length)
        buf.extract_message()
        print("❌ Test 6.1 ECHEC: Exception non levée pour longueur incorrecte.")
    except Exception as e:
        print(f"✅ Test 6.1 OK: Exception levée pour longueur incorrecte. \nMessage : {e}")
    
    try:
        result = buf.extract_message()[1]
        print(f"Test 6 - Message extrait après nettoyage : {result}")
        assert result is None
        print(f"✅ Test 6.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 6.2 OK: Le buffer n'a pas été nettoyé. \nMessage : {w}")


    print("\n\nTest 7 : Vérifie l'extraction d'un message fragmenté reçu en deux parties.")
    # Test 7 : Extraction d'un message en deux temps (incomplet puis complet)
    try:
        msg = "Test 7 - message fragmenté"
        encapsulated = test_encapsulate(msg, "string")
        # On découpe le message en deux parties
        part1 = encapsulated[:len(encapsulated)//2]
        part2 = encapsulated[len(encapsulated)//2:]
        buf.clear()
        print(f"\nTest 7 - Message de base : {msg}")
        print(f"Test 7 - Encapsulation complète : {encapsulated}")
        print(f"Test 7 - Partie 1 envoyée : {part1}")
        buf.append(part1)
        result = buf.extract_message()
        print(f"Test 7 - Résultat après partie 1 : {result}")
        if result[1] is None:
            print("✅ Test 7.1 OK: Aucun message extrait, message incomplet.")
        else:
            print("❌ Test 7.1 ECHEC: Un message a été extrait alors qu'il est incomplet.")
        print(f"Test 7 - Partie 2 envoyée : {part2}")
        buf.append(part2)
        result = buf.extract_message()
        print(f"Test 7 - Résultat après partie 2 : {result}")
        if result[1] == msg:
            print("✅ Test 7.2 OK: Message extrait correctement après réception complète.")
        else:
            print("❌ Test 7.2 ECHEC: Le message extrait n'est pas correct.")
    except Exception as e:
        print(f"❌ Test 7 ECHEC: {e}")


    print("\n\nTest 8 : Vérifie la détection d'un message trop court pour être valide.")
    # test 8 : Message trop court pour être valide
    try:
        buf.clear()
        msg = b'\xAA\xBB\x01\x00\x00\xCC\xDD'  # message trop court
        print(f"\nTest 8 - Message vide ajouté au buffer: {msg}")
        buf.append(msg)
        print(buf.extract_message())
        print("❌ Test 8.1 ECHEC: Exception non levée pour message vide.")
    except Exception as e:
        print(f"✅ Test 8.1 OK: Exception levée pour message trop court. \nMessage : {e}")
    
    try:
        result = buf.extract_message()
        print(f"Test 8 - Message extrait après nettoyage : {result}")
        assert result[1] is None
        print(f"✅ Test 8.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 8.2 OK: Le buffer n'a pas été nettoyé. \nMessage : {w}")


    print("\n\nTest 9 : Vérifie la gestion d'un reliquat dans le buffer (pas de début mais taille non nulle).")
    # Test 9 : Reliquat dans le buffer (pas de START_MARKER mais taille > 0)
    try:
        buf.clear()
        reliquat = b'zzzzzzzz'  # Données sans marqueur de début
        print(f"\nTest 9 - Reliquat ajouté au buffer: {reliquat}")
        buf.append(reliquat)
        result = buf.extract_message()
        print(f"Test 9 - Résultat de l'extraction : {result}")
        print("❌ Test 9 ECHEC: Warning non levé pour reliquat dans le buffer.")
    except Warning as w:
        print(f"✅ Test 9.1 OK: Warning levé pour reliquat dans le buffer. \nMessage : {w}")
    except Exception as e:
        print(f"❌ Test 9.1 ECHEC: Mauvaise exception levée: {e}")
    
    try:
        result = buf.extract_message()
        print(f"Test 9 - Message extrait après nettoyage : {result}")
        assert result[1] is None
        print(f"✅ Test 9.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 9.2 OK: Le buffer n'a pas été nettoyé. \nMessage : {w}")


    print("\n\nTest 10 : Vérifie l'encapsulation d'un message 'picture_ask'.")
    # Test 10 : Encapsulation d'un message 'picture_ask'
    try:
        buf.clear()
        encapsulated = test_encapsulate(None, "picture_ask")
        buf.append(encapsulated)
        print(f"Test 10 - Encapsulation obtenue : {encapsulated}")
        print("✅ Test 10.1 OK: Encapsulation 'picture_ask' réussie.")
    except Exception as e:
        print(f"❌ Test 10.1 ECHEC: {e}")
    try:
        msg_type, msg = buf.extract_message()
        assert msg_type == "picture_ask" and msg == None, f"❌ Test 10.2 ECHEC : Mauvais message lu : {msg_type} / {msg}"
        print(f"✅ Test 10.2 OK: Message bien extrait")
    except Exception as e:
        print(f"❌ Test 10.2 ECHEC : {e}")
        

    print("\n\nTest 11 : Vérifie la réception d'une image complète.")
    


    print("\n==================== Fin du protocole de test ====================\n\n")

    