# Capture ILA de ulpi_luna_acm (console Tcl de Vivado) :
#     source C:/ulpi/Python/ila_capture.tcl
#     capture setup      ;# déclenche sur le PID SETUP (0x2D) reçu
#     capture rxdata     ;# déclenche sur le premier octet de paquet reçu
#     capture rxerror    ;# déclenche sur un RX CMD « RxError »
#     capture reset      ;# déclenche sur le reset USB détecté par LUNA
# Après « capture ... », basculer SW0 de 0 à 1 dans la minute.
# Le CSV est écrit dans C:/ulpi/ila_<mode>_<hhmmss>.csv (seulement si déclenché) :
#     python ulpi_decode.py C:\ulpi\ila_setup.csv

set LTX {C:/ulpi/Python/build/ulpi_luna_acm.ltx}

proc ila_connect {} {
    global LTX
    # current_hw_server / current_hw_target renvoient "" (avec un simple
    # avertissement) quand rien n'est connecté : on teste la valeur.
    catch {open_hw_manager}
    if {[current_hw_server -quiet] eq ""} {
        connect_hw_server
    }
    if {[current_hw_target -quiet] eq "" || ![get_property IS_OPENED [current_hw_target]]} {
        open_hw_target
    }
    set dev [lindex [get_hw_devices xc7a35t*] 0]
    current_hw_device $dev
    set_property PROBES.FILE $LTX $dev
    set_property FULL_PROBES.FILE $LTX $dev
    refresh_hw_device $dev
    return [lindex [get_hw_ilas -of_objects $dev] 0]
}

proc probe {ila name value} {
    set_property TRIGGER_COMPARE_VALUE $value [get_hw_probes $name -of_objects $ila]
}

proc capture {mode} {
    set ila [ila_connect]
    reset_hw_ila $ila
    set_property CONTROL.TRIGGER_CONDITION AND $ila
    set_property CONTROL.TRIGGER_POSITION 400 $ila
    # tout « indifférent » au départ
    foreach p {ulpi_dir ulpi_nxt ulpi_stp ulpi_oe usb_reset usb_connect} { probe $ila $p "eq1'bX" }
    probe $ila ulpi_din  "eq8'hXX"
    probe $ila ulpi_dout "eq8'hXX"
    switch $mode {
        setup {
            probe $ila ulpi_dir "eq1'b1"
            probe $ila ulpi_nxt "eq1'b1"
            probe $ila ulpi_din "eq8'h2D"
        }
        rxdata {
            # n'importe quel octet de paquet reçu (DIR = 1 et NXT = 1)
            probe $ila ulpi_dir "eq1'b1"
            probe $ila ulpi_nxt "eq1'b1"
        }
        rxerror {
            probe $ila ulpi_dir "eq1'b1"
            probe $ila ulpi_nxt "eq1'b0"
            probe $ila ulpi_din "eq8'bXX11XXXX"
        }
        reset {
            probe $ila usb_reset "eq1'bR"
        }
        default { error "mode inconnu : $mode (setup, rxdata, rxerror ou reset)" }
    }
    run_hw_ila $ila
    puts ">>> ILA armée ($mode) : basculez SW0 de 0 à 1 maintenant (1 minute max)"
    # wait_on_hw_ila ne renvoie pas d'erreur à l'expiration : on vérifie qu'il
    # y a des données avant d'écrire quoi que ce soit.
    catch {wait_on_hw_ila -timeout 1 $ila}
    set data [upload_hw_ila_data -quiet $ila]
    if {$data eq ""} {
        puts ">>> PAS DE DÉCLENCHEMENT en 1 minute ($mode) : aucun fichier écrit"
        return
    }
    set csv "C:/ulpi/ila_${mode}_[clock format [clock seconds] -format %H%M%S].csv"
    write_hw_ila_data -force -csv_file $csv $data
    puts ">>> DÉCLENCHÉ ($mode) : capture écrite dans $csv"
}
