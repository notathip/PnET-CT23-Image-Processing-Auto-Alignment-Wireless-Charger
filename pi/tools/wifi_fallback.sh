#!/bin/bash
HOME_CON="Sumlee"; HOME_SSID="Sumlee"
AP_CON="evcharger-ap"
TICK=15            # วิ ต่อรอบเช็ค (ระหว่างรอ ตรวจไฟล์คำขอทุก 1 วิ)
LOST_TO_AP=45      # ต่อ Sumlee ไม่ได้นานเท่านี้ (วิ) -> เปิด AP

DIR=/run/evcharger-wifi
first_boot=0; [ -e "$DIR/mode" ] || first_boot=1
mkdir -p "$DIR"; chmod 1777 "$DIR"; rm -f "$DIR/request"

LOGFILE=/home/notathip/wifi_fallback.log   # journal รีบูตแล้วหาย (ก่อน 12 ต.ค.) -> เขียนไฟล์ด้วย
log() { echo "$(date '+%F %T') $*" | tee -a "$LOGFILE"; }
result() { echo "$1|$2|$(date +%s)" > "$DIR/result.tmp" && mv -f "$DIR/result.tmp" "$DIR/result"; chmod 644 "$DIR/result"; }
active() { timeout 10 nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="wlan0"{print $1}'; }
setmode() { echo "$1" > "$DIR/mode.tmp" && mv -f "$DIR/mode.tmp" "$DIR/mode"; chmod 644 "$DIR/mode"; }

do_switch() {
    local cur="$1"
    if [ "$cur" = "$HOME_CON" ]; then
        result already "ต่อ $HOME_SSID อยู่แล้ว"; return
    fi
    if [ "$cur" != "$AP_CON" ]; then
        result busy "กำลังเปลี่ยนเครือข่ายอยู่ ลองใหม่ในไม่กี่วินาที"; return
    fi
    log "ปุ่มบนเว็บ: สแกนหา $HOME_SSID"
    result scanning "กำลังสแกนหา $HOME_SSID"
    scan=$(timeout 30 iw dev wlan0 scan ap-force 2>&1); rc=$?
    if [ $rc -ne 0 ]; then
        log "สแกน: ผิดพลาด rc=$rc $(echo "$scan" | head -1)"
        result error "สแกนไม่สำเร็จ (rc=$rc) ลองกดใหม่"; return
    fi
    if ! echo "$scan" | grep -q "SSID: ${HOME_SSID}$"; then
        n=$(echo "$scan" | grep -c '^BSS ')
        log "สแกน: ไม่เจอ $HOME_SSID (เห็น WiFi $n วง)"
        result notfound "ไม่เจอ $HOME_SSID (เห็น WiFi $n วง) · เปิด hotspot แล้วกดใหม่"; return
    fi
    log "เจอ $HOME_SSID -> ปิด AP แล้วต่อ hotspot"
    result switching "เจอ $HOME_SSID แล้ว กำลังสลับ — เครื่องนี้จะหลุดจาก AP ให้ไปต่อ hotspot แล้วเปิด athip.local"
    sleep 4                                     # ให้เว็บดึงข้อความนี้ไปโชว์ก่อนหลุด
    timeout 20 nmcli connection down "$AP_CON" >/dev/null 2>&1
    if timeout 45 nmcli connection up "$HOME_CON" >/dev/null 2>&1; then
        log "ต่อ $HOME_SSID สำเร็จ"; lost=0
        result ok "ต่อ $HOME_SSID แล้ว"
    else
        log "ต่อ $HOME_SSID ไม่สำเร็จ -> กลับเป็น AP"
        timeout 45 nmcli connection up "$AP_CON" >/dev/null 2>&1
        result fail "ต่อ $HOME_SSID ไม่สำเร็จ กลับเป็น AP แล้ว"
    fi
}

do_disconnect() {
    local cur="$1"
    if [ "$cur" = "$AP_CON" ]; then
        result already "เป็น AP อยู่แล้ว"; return
    fi
    if [ "$cur" != "$HOME_CON" ]; then
        result busy "กำลังเปลี่ยนเครือข่ายอยู่ ลองใหม่ในไม่กี่วินาที"; return
    fi
    log "ปุ่มบนเว็บ: ตัดการเชื่อมต่อ $HOME_SSID -> เปิด AP"
    result switching "กำลังตัด $HOME_SSID แล้วเปิด AP — เครื่องนี้จะหลุด ให้ไปต่อ WiFi evchargerPnET แล้วเปิด evcharger.pnet"
    sleep 4                                     # ให้เว็บดึงข้อความนี้ไปโชว์ก่อนหลุด
    timeout 20 nmcli connection down "$HOME_CON" >/dev/null 2>&1
    if timeout 45 nmcli connection up "$AP_CON" >/dev/null 2>&1; then
        log "AP เปิดแล้ว (จากปุ่ม)"; since_beat=0; lost=0; setmode ap
        result ok "เปิด AP แล้ว"
    else
        log "เปิด AP ไม่สำเร็จ -> ต่อ $HOME_SSID กลับ"
        timeout 45 nmcli connection up "$HOME_CON" >/dev/null 2>&1
        result fail "เปิด AP ไม่สำเร็จ ต่อ $HOME_SSID กลับแล้ว"
    fi
}

boot_to_ap() {
    local i
    for i in $(seq 10); do
        if timeout 45 nmcli connection up "$AP_CON" >/dev/null 2>&1; then
            log "บูต: เปิด AP แล้ว (ครั้งที่ $i)"; since_beat=0; setmode ap
            return 0
        fi
        sleep 3
    done
    log "บูต: เปิด AP ไม่สำเร็จ -> ให้ลูปหลักลองต่อ"
    return 1
}

lost=0
log "เริ่ม (home=$HOME_CON ap=$AP_CON · บูตเป็น AP · ต่อ $HOME_SSID ด้วยปุ่มบนเว็บ · รอบแรกหลังบูต=$first_boot)"
if [ "$(timeout 10 nmcli -g connection.autoconnect connection show "$HOME_CON" 2>/dev/null)" = "yes" ]; then
    timeout 10 nmcli connection modify "$HOME_CON" connection.autoconnect no \
        && log "ปิด autoconnect ของ $HOME_CON แล้ว" || log "ปิด autoconnect ของ $HOME_CON ไม่สำเร็จ"
fi
[ "$first_boot" = 1 ] && boot_to_ap
while true; do
    cur=$(active)
    if [ "$cur" = "$HOME_CON" ]; then
        lost=0; setmode home
    elif [ "$cur" = "$AP_CON" ]; then
        setmode ap
        n_sta=$(timeout 10 iw dev wlan0 station dump 2>/dev/null | grep -c ^Station)
        [ "$n_sta" != "${last_sta:-x}" ] && log "AP: มีเครื่องต่ออยู่ $n_sta เครื่อง" && last_sta=$n_sta
        since_beat=$((${since_beat:-0} + TICK))
        [ $since_beat -ge 600 ] && since_beat=0 && log "AP: ยังทำงานอยู่ (เครื่องต่อ $n_sta)"
    else
        setmode none
        lost=$((lost + TICK))
        if [ $lost -ge $LOST_TO_AP ]; then
            log "ไม่ได้ต่อ $HOME_SSID มา ${lost} วิ -> เปิด AP"
            if timeout 45 nmcli connection up "$AP_CON" >/dev/null 2>&1; then
                log "AP เปิดแล้ว"; since_beat=0; setmode ap
            else
                log "เปิด AP ไม่สำเร็จ -> ลองใหม่รอบหน้า"
            fi
            lost=0
        fi
    fi
    for i in $(seq $TICK); do
        if [ -e "$DIR/request" ]; then
            req=$(head -c 16 "$DIR/request" 2>/dev/null); rm -f "$DIR/request"
            if [ "$req" = "ap" ]; then do_disconnect "$(active)"; else do_switch "$(active)"; fi
            break
        fi
        sleep 1
    done
done
