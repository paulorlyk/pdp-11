//
// Created by palulukan on 6/11/20.
//

#include "cpu.h"

#include "log.h"

#include <string.h>
#include <stdbool.h>
#include <assert.h>

static struct
{
    cpu_word GPR[8];
    cpu_word PSW;
} cpu;

enum opcode_
{
    _mov        = 0x1000,   // MOV      Move
    _movb       = 0x9000,   // MOVB     Move
    _cmp        = 0x2000,   // CMP      Compare
    _cmpb       = 0xA000,   // CMPB     Compare
    _bit        = 0x3000,   // BIT      Bit Test
    _bitb       = 0xB000,   // BITB     Bit Test
    _bic        = 0x4000,   // BIC      Bit Clear
    _bicb       = 0xC000,   // BICB     Bit Clear
    _bis        = 0x5000,   // BIS      Bit Set
    _bisb       = 0xD000,   // BISB     Bit Set
    _add        = 0x6000,   // ADD      Add
    _sub        = 0xE000,   // SUB      Subtract
    _mul        = 0x7000,   // MUL      Multiply
    _div        = 0x7100,   // DIV      Divide
    _ash        = 0x7200,   // ASH      Arithmetic Shift
    _ashc       = 0x7300,   // ASHC     Arithmetic Shift Combined
    _xor        = 0x7400,   // XOR      Exclusive OR
    _fp         = 0x7500,
    _sys        = 0x7600,
    _sob        = 0x7700,   // SOB      Subtract one and branch if not equal to 0
    _swab       = 0x00C0,   // SWAB     Swap Bytes
    _clr        = 0x0A00,   // CLR      Clear
    _clrb       = 0x8A00,   // CLRB     Clear
    _com        = 0x0A40,   // COM      Complement
    _comb       = 0x8A40,   // COMB     Complement
    _inc        = 0x0A80,   // INC      Increment
    _incb       = 0x8A80,   // INCB     Increment
    _dec        = 0x0AC0,   // DEC      Decrement
    _decb       = 0x8AC0,   // DECB     Decrement
    _neg        = 0x0B00,   // NEG      Negate
    _negb       = 0x8B00,   // NEGB     Negate
    _adc        = 0x0B40,   // ADC      Add Carry
    _adcb       = 0x8B40,   // ADCB     Add Carry
    _sbc        = 0x0B80,   // SBC      Subtract Carry
    _sbcb       = 0x8B80,   // SBCB     Subtract Carry
    _tst        = 0x0BC0,   // TST      Test
    _tstb       = 0x8BC0,   // TSTB     Test
    _ror        = 0x0C00,   // ROR      Rotate Right
    _rorb       = 0x8C00,   // RORB     Rotate Right
    _rol        = 0x0C40,   // ROL      Rotate Left
    _rolb       = 0x8C40,   // ROLB     Rotate Left
    _asr        = 0x0C80,   // ASR      Arithmetic Shift Right
    _asrb       = 0x8C80,   // ASRB     Arithmetic Shift Right
    _asl        = 0x0CC0,   // ASL      Arithmetic Shift Left
    _aslb       = 0x8CC0,   // ASLB     Arithmetic Shift Left
    _mark       = 0x0D00,   // MARK     Used as part of the standard PDP-11 subroutine return convention.
    _mtps       = 0x8D00,   // MFPS     Move Byte from PSW
    _mfpi       = 0x0D40,   // MFPI     Move from previous instruction space
    _mfpd       = 0x8D40,   // MFPD     Move from previous data space
    _mtpi       = 0x0D80,   // MTPI     Move to previous instruction space
    _mtpd       = 0x8D80,   // MTPD     Move to previous data space
    _sxt        = 0x0DC0,   // SXT      Sign Extend
    _mfps       = 0x8DC0,   // MTPS     Move Byte to PSW
    _br         = 0x0100,   // BR       Branch (Unconditional)
    _bne        = 0x0200,   // BNE      Branch if not equal (to zero)
    _beq        = 0x0300,   // BEQ      Branch if equal (to zero)
    _bge        = 0x0400,   // BGE      Branch if greater than or equal (to zero)
    _blt        = 0x0500,   // BLT      Branch if less than (zero)
    _bgt        = 0x0600,   // BGT      Branch if greater than (zero)
    _ble        = 0x0700,   // BLE      Branch if less than or equal (to zero)
    _bpl        = 0x8000,   // BPL      Branch if plus
    _bmi        = 0x8100,   // BMI      Branch if minus
    _bhi        = 0x8200,   // BHI      Branch if higher
    _blos       = 0x8300,   // BLOS     Branch if lower or same
    _bvc        = 0x8400,   // BVC      Branch if V bit clear
    _bvs        = 0x8500,   // BVS      Branch if V bit set
    _bcc        = 0x8600,   // BCC      Branch if carry clear
    _bcs        = 0x8700,   // BCS      Branch if carry set

    // 0001DD   JMP      Jump
    // 004RDD   JSR      Jump to Subroutine
    // 00020R   RTS      Return from Subroutine

    // 000000   HALT     Halt
    // 000001   WAIT     Wait for Interrupt
    // 000002   RTI      Return from Interrupt
    // 000003   BPT      Breakpoint Trap
    // 000004   IOT      I/O Trap
    // 000005   RESET    Sends INIT on the UNIBUS for 10ms.
    // 000006   RTT      Return from Interrupt
    // 000007   MFPT     Move From Processor (PDP-11/44 ONLY)

    // 00023N   SPL      Set priority level

    // 000240   C        Clear selected condition code bits
    // 000241   CLC      Clear C
    // 000242   ClV      Clear V
    // 000244   ClZ      Clear Z
    // 000250   ClN      Clear N
    // 000257   CCC      Clear all condition code bits

    // 000260   S        Set selected condition codes
    // 000261   SEC      Set C
    // 000262   SEV      Set V
    // 000264   SEZ      Set Z
    // 000270   SEN      Set N
    // 000277   SCC      Set all condition codes

    // 007000   CSM      Call to Supervisor Mode (PDP-11/44 only)

    // 076600   MED      Maintenance, Exam, and Dep

    // 104000   EMT      Emulator Trap
    //    ...
    // 104377   EMT      Emulator Trap
    // 104400   TRAP     Trap
    //    ...
    // 104777   TRAP     Trap

    // 170003   LOUB     Load Microbreak Register
    // 170004   MNS      Maintenance normalization shift
    // 170005   MPP      Maintenance Partial Product
};

enum addressingMode_
{
    _reg               = 0, // The operand is in Rn
    _reg_deferred      = 1, // Rn contains the address of the operand
    _auto_inc          = 2, // Rn contains the address of the operand, then increment Rn by 1 in byte command and by 2 in word command
    _auto_inc_deferred = 3, // Rn contains the address of the address of the operand, then increment Rn by 2
    _auto_dec          = 4, // Decrement Rn by 1 in byte command and by 2 in word command, then use it as the address of the operand
    _auto_dec_deferred = 5, // Decrement Rn by 2, then use it as the address of the address of the operand
    _index             = 6, // Rn + X is the address of the operand
    _index_deferred    = 7  // Rn + X is the address of the address of the operand
};

struct instruction_
{
    enum opcode_ opcode;
    enum addressingMode_ srcMode;
    int src;
    enum addressingMode_ dstMode;
    int dst;
    int reg;
    int8_t offset;
};

static cpu_word signExtend_(cpu_word w, bool byte)
{
    return byte ? (w & 0x00FF | ((w & 0x8000) ? ~0xFF : 0)) : w;
}

static cpu_word read_(cpu_addr addr, bool byte)
{
    cpu_word data = mem_read_physical(addr);

    if(byte)
        data = data >> (8 * (addr & 1U)) & ~0xFF;

    return signExtend_(data, byte);
}

static void write_(cpu_addr addr, bool byte, cpu_word data)
{
//    if(byte)
//        data &= 0xFF;

    mem_write_physical(addr, byte, data);
}

static cpu_word fetchPC_(void)
{
    cpu_word res = read_(cpu.GPR[7], false);
    cpu.GPR[7] += 2;
    return res;
}

static void decode_(cpu_word inst, struct instruction_* pInst)
{
    // TODO: Debug
    pInst->opcode = 0xFFFFFF;

    if((inst & 0x7000) == 0x0000)
    {
        if(((inst & 0xFFC0) == 0x00C0) || ((inst & 0x7FC0) >= 0x0A00 && (inst & 0x7FC0) <= 0x0DC0))
        {
            // Single-operand instructions
            //  15  14 11  10   6   5  3   2      0
            // [B] [0001] [Opcode] [Mode] [Register]
            //
            // Opcode   Mnemonic
            // 0003     SWAB
            // 010      CLR / CLRB
            // 011      COM / COMB
            // 012      INC / INCB
            // 013      DEC / DECB
            // 014      NEG / NEGB
            // 015      ADC / ADCB
            // 016      SBC / SBCB
            // 017      TST / TSTB
            // 020      ROR / RORB
            // 021      ROL / ROLB
            // 022      ASR / ASRB
            // 023      ASL / ASLB
            // 024      MARK / MTPS
            // 025      MFPI / MFPD
            // 026      MTPI / MTPD
            // 027      SXT / MFPS

            DEBUG("Decoder: Single-operand instruction");

            pInst->opcode = inst & 0xFFC0;
            pInst->dstMode = (inst & 0x0038) >> 3;
            pInst->dst = (inst & 0x0007) >> 0;
        }
        else if((inst & 0x7800) == 0)
        {
            if(inst > 0x00FF)
            {
                // Conditional branch instructions
                //  15      11  10   8   7    0
                // [x 0 0 0 0] [Opcode] [Offset]
                //
                // Opcode   Mnemonic
                // 0000xx   <system instructions>
                // 1000xx   BPL
                // 0004xx   BR
                // 1004xx   BMI
                // 0010xx   BNE
                // 1010xx   BHI
                // 0014xx   BEQ
                // 101400   BLOS
                // 0020xx   BGE
                // 1020xx   BVC
                // 0024xx   BLT
                // 1024xx   BVS
                // 0030xx   BGT
                // 1030xx   BCC or BHIS
                // 0034xx   BLE
                // 1034xx   BCS or BLO

                DEBUG("Decoder: Conditional branch instruction");

                pInst->opcode = inst & 0xFF00;
                pInst->offset = inst & 0x00FF;
            }
            else
            {
                // System instructions

                DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
                assert(false);
            }
        }
        else
        {
            // Single-operand instructions
            //  15  14 11  10   6   5  3   2      0
            // [B] [0001] [Opcode] [Mode] [Register]
            //
            // Opcode   Mnemonic
            // 004r     JSR
            // 104x     EMT
            //

            DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
            assert(false);
        }
    }
    else if((inst & 0x7000) == 0x7000)
    {
        if((inst & 0x8000) == 0)
        {
            // Double-operand instructions with register source operand
            //  15 12  11   9   8      6   5  3   2      0
            // [0111] [Opcode] [Register] [Mode] [Dest/Src]
            //
            // Opcode   Mnemonic
            // 00       MUL
            // 01       DIV
            // 02       ASH
            // 03       ASHC
            // 04       XOR
            // 05       Floating point operations
            // 06       System instructions
            // 07       SOB

            DEBUG("Decoder: Double-operand instruction with register source operand");

            pInst->opcode = inst & 0x7000;
            pInst->reg = (inst & 0x01C0) >> 6;
            pInst->srcMode = (inst & 0x0038) >> 3;
            pInst->src = (inst & 0x0007) >> 0;
        }
        else
        {
            DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
            assert(false);
        }
    }
    else
    {
        // Double-operand instructions
        //  15  14   12  11 9   8    6   5  3   2         0
        // [B] [Opcode] [Mode] [Source] [Mode] [Destination]
        //
        // Opcode   Mnemonic
        // 01       MOV / MOVB
        // 02       CMP / CMPB
        // 03       BIT / BITB
        // 04       BIC / BICB
        // 05       BIS / BISB
        // 06       ADD / SUB

        DEBUG("Decoder: Double-operand instruction");

        pInst->opcode = inst & 0x7000;
        pInst->srcMode = (inst & 0x0E00) >> 9;
        pInst->src = (inst & 0x01C0) >> 6;
        pInst->dstMode = (inst & 0x0038) >> 3;
        pInst->dst = (inst & 0x0007) >> 0;
    }

    // TODO: Debug
    if(pInst->opcode == 0xFFFFFF)
    {
        DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
        assert(false);
    }
}

static bool calculateOperandAddress_(enum addressingMode_ mode, int reg, bool byte, cpu_addr *pAddr)
{
    assert(mode >= _reg && mode <= _index_deferred);
    assert(reg >= 0 && reg < 8);

    cpu_word *pRn = cpu.GPR + reg;

    switch(mode)
    {
        default:
        case _reg:
            *pAddr = 0;
            return false;

        case _reg_deferred:
            *pAddr = *pRn;
            break;

        case _auto_inc:
            *pAddr = *pRn;
            *pRn += 1 + !byte;
            break;

        case _auto_inc_deferred:
            *pAddr = read_(*pRn, false);
            *pRn += 2;
            break;

        case _auto_dec:
            *pRn -= 1 + !byte;
            *pAddr = *pRn;
            break;

        case _auto_dec_deferred:
            *pRn -= 1 + !byte;
            *pAddr = read_(*pRn, false);
            break;

        case _index:
        {
            // Make sure that index was fetched before reading register value
            // This is important in case if register itself is PC
            cpu_word X = fetchPC_();
            *pAddr = *pRn + X;
            break;
        }

        case _index_deferred:
        {
            // Make sure that index was fetched before reading register value
            // This is important in case if register itself is PC
            cpu_word X = fetchPC_();
            *pAddr = read_(*pRn + X, false);
            break;
        }
    }

    return true;
}

static cpu_word fetchOperand_(enum addressingMode_ mode, int reg, bool byte, cpu_addr *pAddr)
{
    if(calculateOperandAddress_(mode, reg, byte, pAddr))
        return read_(*pAddr, byte);

    return signExtend_(cpu.GPR[reg], byte);
}

static void storeDestResult_(enum addressingMode_ mode, int reg, bool byte, cpu_addr addr, cpu_word data)
{
    assert(mode >= _reg && mode <= _index_deferred);
    assert(reg >= 0 && reg < 8);

    if(mode == _reg)
        cpu.GPR[reg] = data;
    else
        write_(addr, byte, data);
}

static void setFlags_(bool n, bool z, bool v, bool c)
{
    cpu.PSW = (cpu.PSW & ~0x000F) | (n << 3) | (z << 2) | (v << 1) | (c << 0);
}

void cpu_init(cpu_word R7)
{
    memset(&cpu, 0, sizeof(cpu));

    cpu.GPR[7] = R7;
}

void cpu_run(void)
{
    cpu_word instWord = fetchPC_();
    DEBUG("Instruction: 0%06o", instWord);

    struct instruction_ inst = {0};
    decode_(instWord, &inst);

    // TODO: 3 word instruction
    if((inst.srcMode == _index || inst.srcMode == _index_deferred) && (inst.dstMode == _index || inst.dstMode == _index_deferred))
    {
        DEBUG("Double index instruction - not sure how to execute");
        assert(false);
    }

    // TODO: 3 word instruction
    if(((inst.srcMode == _index || inst.srcMode == _index_deferred) && inst.dst == 7) || ((inst.dstMode == _index || inst.dstMode == _index_deferred) && inst.src == 7))
    {
        DEBUG("Index + PC or PC + index instruction - not sure how to execute");
        assert(false);
    }

    // TODO: 3 word instruction
    if(inst.dst == 7 && inst.src == 7)
    {
        DEBUG("PC + PC instruction - not sure how to execute");
        assert(false);
    }

    cpu_word operand1 = 0;
    cpu_addr operand1Addr = 0;

    cpu_word operand2 = 0;
    cpu_addr operand2Addr = 0;

    bool byteFlag = inst.opcode & 0x8000;

    switch(inst.opcode)
    {
        case _mov:
        case _movb:
        {
            DEBUG("MOV / MOVB");
            operand1 = fetchOperand_(inst.srcMode, inst.src, byteFlag, &operand1Addr);
            calculateOperandAddress_(inst.dstMode, inst.dst, byteFlag, &operand2Addr);
            storeDestResult_(inst.dstMode, inst.dst, byteFlag, operand2Addr, operand1);
            setFlags_(operand1 & 0x8000, operand1 == 0, 0, PSW_GET_C(cpu.PSW));
            break;
        }

        case _cmp: assert(false);
        case _cmpb: assert(false);

        case _bit: assert(false);
        case _bitb: assert(false);

        case _bic: assert(false);
        case _bicb: assert(false);

        case _bis: assert(false);
        case _bisb: assert(false);

        case _add: assert(false);

        case _sub: assert(false);

        case _mul: assert(false);

        case _div: assert(false);

        case _ash: assert(false);

        case _ashc: assert(false);

        case _xor: assert(false);

        case _fp: assert(false);

        case _sys: assert(false);

        case _sob: assert(false);

        case _swab: assert(false);

        case _clr: assert(false);
        case _clrb: assert(false);

        case _com: assert(false);
        case _comb: assert(false);

        case _inc: assert(false);
        case _incb: assert(false);

        case _dec: assert(false);
        case _decb: assert(false);

        case _neg: assert(false);
        case _negb: assert(false);

        case _adc: assert(false);
        case _adcb: assert(false);

        case _sbc: assert(false);
        case _sbcb: assert(false);

        case _tst:
        case _tstb:
        {
            DEBUG("TST / TSTB");
            operand1 = fetchOperand_(inst.dstMode, inst.dst, byteFlag, &operand1Addr);
            setFlags_(operand1 & 0x8000, operand1 == 0, 0, 0);
            break;
        }

        case _ror: assert(false);
        case _rorb: assert(false);

        case _rol: assert(false);
        case _rolb: assert(false);

        case _asr: assert(false);
        case _asrb: assert(false);

        case _asl: assert(false);
        case _aslb: assert(false);

        case _mark: assert(false);

        case _mtps: assert(false);

        case _mfpi: assert(false);

        case _mfpd: assert(false);

        case _mtpi: assert(false);

        case _mtpd: assert(false);

        case _sxt: assert(false);

        case _mfps: assert(false);

        case _br: assert(false);

        case _bne: assert(false);

        case _beq: assert(false);

        case _bge: assert(false);

        case _blt: assert(false);

        case _bgt: assert(false);

        case _ble: assert(false);

        case _bpl:
        {
            DEBUG("BPL");
            if(!PSW_GET_N(cpu.PSW))
                cpu.GPR[7] += (int)inst.offset * 2;
            break;
        }

        case _bmi: assert(false);

        case _bhi: assert(false);

        case _blos: assert(false);

        case _bvc: assert(false);

        case _bvs: assert(false);

        case _bcc: assert(false);

        case _bcs: assert(false);

        default:
            DEBUG("UNIMPLEMENTED INSTRUCTION: 0%06o", instWord);
            assert(false);
            break;
    }
}
