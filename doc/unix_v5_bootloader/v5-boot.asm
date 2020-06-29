; Disasembled and partially comented boot sector from unix V5 disk image.
; Sources of another bootloader found on that disk was used as a refference.
; Final addresses after relocation are shown (look at the beginning of the entry point).

000000: 000407          br start
000002: 000676          br 37777777600
000004: 000000          halt
000006: 000000          halt
000010: 000000          halt
000012: 000000          halt
000014: 000000          halt
000016: 000001          wait

DL11_RCSR = 177560
DL11_RBUF = 177562
DL11_XCSR = 177564
DL11_XBUF = 177566

RK11_RKDA = 177412

core = 24.
.. = [core*2048.]-512.

; start - entry point:
;   - if PC > SP - continue
;   - else copy program code to right after SP
;       - if first word is br opcode - skip first 16 bytes while copyng
;   - set all memory from 0 to SP to 0
                        start:
137000: 012706 137000       mov $.., SP
137004: 010601              mov SP, R1
137006: 005000              clr R0
137010: 020701              cmp PC, R1
137012: 103012              bhis 2f
137014: 021027 000407       cmp (R0), $407
137020: 001002              bne 1f
137022: 012700 000020       mov $20, R0
                        1:
137026: 012021              mov (R0)+, (R1)+
137030: 020127 140000       cmp R1, $end
137034: 103774              blo 1b
137036: 000116              jmp (SP)
                        2:
137040: 005020              clr (R0)+
137042: 020006              cmp R0, SP
137044: 103775              blo 2b

; Print '@'
137046: 012705 137620       mov $putc, R5
137052: 012700 000100       mov $'@, R0
137056: 004715              jsr PC, (R5)

; Read names
137060: 012702 136046       mov $names, R2          ; Load address of the names buffer into R2
                        3:
137064: 010201              mov R2, R1              ; Update R1 with the start pointer of the current name
                        1:
137066: 004767 000346       jsr PC, getc            ; Read character from terminal - R0 holds result
137072: 020027 000012       cmp R0, $'\n            ; Check if '\n'
137076: 001412              beq 1f                  ; Break the loop if '\n'
137100: 020027 000057       cmp R0, $'/             ; Check if '/'
137104: 001402              beq 2f                  ; Do buffer advancing if '/'
137106: 110021              movb R0, (R1)+          ; Store character into names buffer
137110: 000766              br 1b                   ; Loop back to reading new character
                        2:
137112: 020201              cmp R2, R1              ; Check if this is the first character
137114: 001764              beq 1b                  ; And if so - loop back to reading new character
137116: 062702 000016       add $14., R2            ; Skip 14 words from the names buffer
137122: 000760              br 3b                   ; Loop back to updating start pointer of the name
                        1:
137124: 012702 136046       mov $names, R2
137130: 012700 000001       mov $1, R0              ; Read inode #1?
                        1:
137134: 005067 176702       clr bno
137140: 004767 000112       jsr PC, iget
137144: 005712              tst (R2)                ; First characher of the name is '\0' - last name
137146: 001430              beq 1f                  ; Jump if so...
                        2:
137150: 004767 000152       call rmblk              ; Read block; rmblk may skip next instruction on return
137154: 000711                  br start            ; Fail? Start over again!
137156: 012701 135040       mov $buf, R1            ; R1 = &buf
                        3:
137162: 010203              mov R2, R3              ; R3 = R2 = &names
137164: 010104              mov R1, R4              ; R4 = R1 = &buf
137166: 062701 000020       add $16., R1            ; R1 += 16
137172: 005724              tst (R4)+
137174: 001411              beq 5f                  ; Fail check if buf[R4++] == 0
                        4:
137176: 122324              cmpb (R3)+, (R4)+
137200: 001007              bne 5f                  ; Fail check if names[R3++] != buf[R4++]
137202: 020401              cmp R4, R1              ; Did we check 16 characters?
137204: 103774              bcs 4b                  ; Loop back if no
137206: 016100 177760       mov $-16.(R1), R0       ; Name matched? Or not? Load next block number into R0?
137212: 062702 000016       add $14., R2            ; Move to next name
137216: 000746              br 1b                   ; Read next inode...
                        5:
137220: 020127 136040       cmp R1, $buf + 512      ; End of the block buffer?
137224: 103756              bcs 3b                  ; No, continue checking
137226: 000750              br 2b                   ; Yes, near the end. go back and read a new one
                        1:
137230: 005002              clr R2
                        2:
137232: 004767 000070       call rmblk
137236: 000555                  br jump_to_boot
137240: 012701 135040       mov $buf, R1            ; Copy buffer to location 0
                        1:
137244: 012122              mov (R1)+, (R2)+
137246: 020127 136040       cmp R1, $buf + 512
137252: 103774              bcs 1b
137254: 000766              br 2b

; R0 - inode number?
                        iget:
137256: 062700 000037       add $31., R0            ; R0 += 31
137262: 010005              mov R0, R5              ; R5 = R0
137264: 072027 177774       ash $-4., R0            ; R0 >>= 4
137270: 004767 000132       call rblk               ; rblk(R0)
137274: 042705 177760       bic $177760, R5         ; R5 &= 0x000F
137300: 072527 000005       ash $5, R5              ; R5 <<= 5
137304: 062705 135040       add $buf, R5            ; R5 += &buf
137310: 012704 135000       mov $inod, R4           ; Copy inode data from buf to inod
                        1:
137314: 012524              mov (R5)+, (R4)+
137316: 020427 135030       cmp R4, $addr + 16
137322: 103774              blo 1b
137324: 000207              rts PC

                        rmblk:
137326: 062716 000002       add $2, (SP)            ; Increment return address to skip one instruction
137332: 016700 176504       mov $bno, R0
137336: 005267 176500       inc bno                 ; Increment bno
137342: 032767 010000 175430 bit $LRG, mode
137350: 001007              bne 1f                  ; Jump if mode is negative? (mode & 0x8000) != 0?
137352: 006300              asl R0
137354: 016000 135010       mov addr(R0), R0        ; Load inode addr into R0
137360: 001022              bne rblk                ; Read block if R0 != 0
                        2:
137362: 162716 000002       sub $2, (SP)            ; Restore return address
137366: 000207              rts PC
                        1:
137370: 005046              clr -(SP)
137372: 110016              movb R0, (SP)
137374: 105000              clrb R0
137376: 000300              swab R0
137400: 006300              asl R0
137402: 016000 135010       mov addr(R0), R0
137406: 001765              beq 2b
137410: 004767 000012       call rblk
137414: 012600              mov (SP)+, R0
137416: 006300              asl R0
137420: 016000 135040       mov buf(R0), R0
137424: 001756              beq 2b

; rblk - read block from disk
; R0 - block number
                        rblk:
137426: 010067 176412       mov R0, rxblk           ; Store block numer to rxblk location
137432: 000475              br rkblk                ; Jump to disk read

                        const_table:
137434: 135040              .word $buf
137436: 177400              .word $-256.

; getc - read character from terminal, tolower() it and echo it back
; R0 will contain read and processed character
; R5 should hold address of putc
                        getc:
137440: 105737 177560       tstb @DL11_RCSR         ; Check RCVR_DONE bit
137444: 002375              bge getc                ; Loop untill DL11 received something
137446: 016700 040110       mov DL11_RBUF, R0       ; Read received byte to R0
137452: 042700 177600       bic $177600, R0         ; Zero out all but first 7 bits
137456: 020027 000101       cmp	R0, $'A             ; Add 32 if character is in between 'A' and 'Z' - effective tolower()
137462: 103405              blo 1f
137464: 020027 000132       cmp R0, $'Z
137470: 101002              bhi 1f
137472: 062700 000040       add $40, R0
                        1:
137476: 020027 000015       cmp R0, $'\r            ; If not '\r' - print chagacter and return from subroutine
137502: 001002              bne putc_replace_lf_with_crlf
137504: 012700 000012       mov $'\n, R0            ; Otherwice replace charanter with '\n' and print
                        putc_replace_lf_with_crlf:
137510: 020027 000012       cmp R0, $'\n            ; Skip if not '\n'
137514: 001005              bne 1f
137516: 012700 000015       mov $'\r, R0            ; If '\n' - substitute character with '\r'
137522: 004715              jsr PC, (R5)            ; Print - R5 should be holding address of putc
137524: 012700 000012       mov $'\n, R0            ; Restore original '\n' character
                        1:
137530: 105767 040030       tstb DL11_XCSR          ; $040030(PC) Check XMIT_RDY bit
137534: 100375              bpl 1b                  ; Loop untill transmitter is ready
137536: 010067 040024       mov R0, DL11_XBUF       ; Write character to transmitter buffer register
137542: 000207              rts PC

                        unknown0:
137544: 117600 000000       movb @0(SP), R0
137550: 001403              beq 1f
137552: 004715              jsr PC, (R5)
137554: 005216              inc (SP)
137556: 000772              br unknown0
                        1:
137560: 062716 000002       add $2, (SP)
137564: 042716 000001       bic $1, (SP)
137570: 000207              rts PC

                        jump_to_boot:
137572: 005000              clr R0
137574: 021027 000407       cmp (R0), $407
137600: 001004              bne 2f
                        1:
137602: 016020 000020       mov 20(R0), (R0)+
137606: 020006              cmp R0, R6
137610: 103774              bcs 1b:
                        2:
137612: 012746 137000       mov $start, -(SP)
137616: 005007              clr PC

; putc - print character in R0 to terminal
; Must be called trough jsr PC, (R5)
                        putc:
137620: 000733              br putc_replace_lf_with_crlf

; Never used
137622: 000706              br getc

; Never used
137624: 000747              br unknown0

; rkblk - read block (256 words, 512 bytes) from disk (RK11 controller)
; Linear block number (a-la LBA?) is taken from rxblk
                        rkblk:
137626: 016701 176212       mov rxblk, R1               ; Load sector number to R1
137632: 005000              clr R0                      ; R0 = 0
137634: 071027 000014       div $12., R0                ; R0 = R1 / 12; R1 = R1 % 12
137640: 072027 000004       ash $4., R0                 ; R0 <<= 4; R0 now holds Drive + Cylinder + Surface (track ddress); R1 is sector address
137644: 050100              bis R1, R0                  ; R0 = R1 | R0; Combine track and sector addresses
137646: 012701 177412       mov $RK11_RKDA, R1
137652: 010011              mov R0, (R1)                ; RKDA = R0
137654: 016741 177554       mov const_table, -(R1)      ; RKBA = buf
137660: 016741 177552       mov const_table + 1, -(R1)  ; RKWC = -256
137664: 012741 000005       mov $5, -(R1)               ; RKCS = Read & Go
                        1:
137670: 105711              tstb (R1)                   ; Check RDY bit in RKCS
137672: 100376              bpl 1b                      ; Loop untill RDY bit is set
137674: 000207              rts PC

47624 = 0135010

end:                            ; 0137674
    inod = 47616.               ; 0135000
        mode = inod
        addr = inod + 8.
    buf = inod + 32.            ; 0135040
    bno = buf + 514.            ; 0136042
    rxblk = bno + 2             ; 0136044
    names = rxblk + 2           ; 0136046
