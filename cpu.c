//
// Created by palulukan on 6/11/20.
//

#include "cpu.h"

#include "log.h"

#include <string.h>
#include <stdbool.h>
#include <assert.h>

typedef uint32_t operand_addr;

#define OPERAND_TYPE_REG    (1U << 16)
#define OPERAND_TYPE_BYTE   (1U << 17)

#define SIGN_EXTEND_BYTE(b) (((b) & 0xFF) ^ ((~0xFF) * (((b) & 0x0080) != 0)))   // Assume two's complement integers

#define BRANCH_IF(cond, offset)             \
    do                                      \
    {                                       \
        if((cond))                          \
            cpu.GPR[7] += (offset) * 2;     \
    } while(0)                              \

#define CALC_V(a, b, r)    ((((a) ^ (b)) & 0x8000) && !(((b) ^ (r)) & 0x8000))

static struct
{
    const char *arrOpcodeNameLUT[0x10000];
    const char *arrRegNameLUT[8];
    const char *arrAddrModePrefixLUT[8];
    const char *arrAddrModeSuffixLUT[8];

    bool bDisassemblyOutput;

    device_handle device;

    cpu_word *GPR;
    cpu_word PSW;

    cpu_word regSet[2][8];
    cpu_word lastSP[4];
} cpu;

enum _opcode
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
    _div        = 0x7200,   // DIV      Divide
    _ash        = 0x7400,   // ASH      Arithmetic Shift
    _ashc       = 0x7600,   // ASHC     Arithmetic Shift Combined
    _xor        = 0x7800,   // XOR      Exclusive OR
    _fp         = 0x7A00,
    _sys        = 0x7C00,
    _sob        = 0x7E00,   // SOB      Subtract one and branch if not equal to 0
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
    _jmp        = 0x0040,   // JMP      Jump
    _jsr        = 0x0800,   // JSR      Jump to Subroutine
    _rts        = 0x0080,   // RTS      Return from Subroutine
    _reset      = 0x0005,   // RESET    Sends INIT on the UNIBUS for 10ms
    _spl        = 0x0098,   // SPL      Set priority level
    _rti        = 0x0002,   // RTI      Return from Interrupt

    // 000000   HALT     Halt
    // 000001   WAIT     Wait for Interrupt
    // 000003   BPT      Breakpoint Trap
    // 000004   IOT      I/O Trap
    // 000006   RTT      Return from Interrupt
    // 000007   MFPT     Move From Processor (PDP-11/44 ONLY)

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

    // 104000   EMT      Emulator Trap
    //    ...
    // 104377   EMT      Emulator Trap
    // 104400   TRAP     Trap
    //    ...
    // 104777   TRAP     Trap

    // 007000   CSM      Call to Supervisor Mode (PDP-11/44 only)

    // 076600   MED      Maintenance, Exam, and Dep

    // 170003   LOUB     Load Microbreak Register
    // 170004   MNS      Maintenance normalization shift
    // 170005   MPP      Maintenance Partial Product
};

enum _addressingMode
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

enum _instructionType
{
    _instt_unknown,
    _instt_single_op,
    _instt_double_op,
    _instt_branch,
    _instt_rts,
    _instt_spl,
    _instt_double_op_reg_src,
    _instt_system
};

struct _instruction
{
    enum _instructionType type;
    enum _opcode opcode;
    enum _addressingMode srcMode;
    int src;
    enum _addressingMode dstMode;
    int dst;
    int reg;
    int offset;
};

static void _trap(cpu_word vec);

static cpu_word _signExtend(cpu_word w, bool bByte)
{
    return bByte ? SIGN_EXTEND_BYTE(w) : w;
}

static bool _read(cpu_addr addr, bool bByte, cpu_space space, cpu_mode mode, cpu_word *data)
{
    uint32_t w = mem_read(addr & ~1U, space, mode);
    if(w & MEM_HAS_ERR)
    {
        DEBUG("CPU MEM READ ERROR: 0x%08x", w);

        // TODO: Update CPU Error Register
        _trap(4);
        return false;
    }

    if(bByte)
        w = w >> (8 * (addr & 1U)) & 0xFF;

    *data = _signExtend(w, bByte);

    return true;
}

static bool _write(cpu_addr addr, bool bByte, cpu_word data, cpu_space space, cpu_mode mode)
{
//    if(bByte)
//        data &= 0xFF;

    uint32_t res = mem_write(addr, space, mode, bByte, data);
    if(res & MEM_HAS_ERR)
    {
        DEBUG("CPU MEM WRITE ERROR: 0x%08x", res);

        // TODO: Update CPU Error Register
        _trap(4);
        return false;
    }

    return true;
}

static int _fetchPC(cpu_word* data)
{
    if(cpu.GPR[7] & 1)
    {
        // TODO: Boundary error trap
        assert(false);

        return 0;
    }

    if(!_read(cpu.GPR[7], false, cpu_space_I, PSW_GET_CUR_MODE(cpu.PSW), data))
        return false;

    cpu.GPR[7] += 2;

    return 2;
}

static void _decode(cpu_word inst, struct _instruction* pInst)
{
    // TODO: Debug
    pInst->opcode = 0xFFFFFF;

    pInst->type = _instt_unknown;

    if((inst & 0x7000) == 0x0000)   // x000 xxxx xxxx xxxx
    {
        if((inst & 0x7800) == 0x0800)    // x000 1xxx xxxx xxxx
        {
            if((inst & 0x7E00) == 0x0800)    // x000 100x xxxx xxxx
            {
                if((inst & 0xFE00) == 0x0800)   // 0000 100x xxxx xxxx
                {
                    // JSR
                    //  15    9   8     6   5  3   2      0
                    // [0000100] [LinkReg] [Mode] [Register]

                    //DEBUG("Decoder: Jsr instruction");

                    pInst->type = _instt_double_op;
                    pInst->opcode = inst & 0xFE00;
                    pInst->srcMode = _reg;
                    pInst->src = (inst & 0x01C0) >> 6;
                    pInst->dstMode = (inst & 0x0038) >> 3;
                    pInst->dst = (inst & 0x0007) >> 0;
                }
                else    // 1000 100x xxxx xxxx
                {
                    // EMT / TRAP
                    //  15    9   8        7  0
                    // [1000100] [Opcode] [NNNN]
                    //
                    // Opcode   Mnemonic
                    // 1040     EMT
                    // 1044     TRAP

                    DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
                    assert(false);
                }
            }
            else if((inst & 0x7E00) == 0x0E00)    // x000 111x xxxx xxxx
            {
                DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
                assert(false);
            }
            else    // x000 1IIx xxxx xxxx; II != 0 && II != 3
            {
                // Single-operand instructions
                //  15  14 11  10   6   5  3   2      0
                // [B] [0001] [Opcode] [Mode] [Register]
                //
                // Opcode   Mnemonic
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

                //DEBUG("Decoder: Single-operand instruction");

                pInst->type = _instt_single_op;
                pInst->opcode = inst & 0xFFC0;
                pInst->dstMode = (inst & 0x0038) >> 3;
                pInst->dst = (inst & 0x0007) >> 0;
            }
        }
        else if((inst & 0x7800) == 0x0000)   // x000 0xxx xxxx xxxx
        {
            if((inst & 0xFF00) == 0x0000)   // 0000 0000 xxxx xxxx
            {
                if((inst & 0xFFC0) == 0x0040 || (inst & 0xFFC0) == 0x00C0)  // 0000 0000 IIxx xxxx; II == 1 || II == 3
                {
                    // Single-operand instructions
                    //  15            8   7    6   5  3   2      0
                    // [0 0 0 0 0 0 0 0] [Opcode] [Mode] [Register]
                    //
                    // Opcode   Mnemonic
                    // 01       JMP
                    // 03       SWAB

                    //DEBUG("Decoder: JMP/SWAB instruction");

                    pInst->type = _instt_single_op;
                    pInst->opcode = inst & 0xFFC0;
                    pInst->dstMode = (inst & 0x0038) >> 3;
                    pInst->dst = (inst & 0x0007) >> 0;
                }
                else if((inst & 0xFFF8) == 0x0080 || (inst & 0xFFF8) == 0x0098)  // 0000 0000 100I Ixxx; II = 0 || II == 3
                {
                    // RTS
                    // SPL
                    //  15                      3   2      0
                    // [0 0 0 0 0 0 0 0 1 0 0 0 0] [Register]

                    //DEBUG("Decoder: RTS/SPL instruction");

                    pInst->type = (inst & 0xFFF8) == 0x0080 ? _instt_rts : _instt_spl;
                    pInst->opcode = inst & 0xFFF8;
                    pInst->dst = (inst & 0x0007) >> 0;
                }
                else
                {
                    // System instructions

                    //DEBUG("Decoder: System instruction");

                    pInst->type = _instt_system;
                    pInst->opcode = inst;
                }
            }
            else    // B000 0III xxxx xxxx; B != 0 && III != 0
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

                //DEBUG("Decoder: Conditional branch instruction");

                pInst->type = _instt_branch;
                pInst->opcode = inst & 0xFF00;
                pInst->offset = SIGN_EXTEND_BYTE(inst);
            }
        }
    }
    else if((inst & 0x7000) == 0x7000)  // x111 xxxx xxxx xxxx
    {
        if((inst & 0xF000) == 0x7000)    // 0111 xxxx xxxx xxxx
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

            //DEBUG("Decoder: Double-operand instruction with register source operand");

            pInst->type = _instt_double_op_reg_src;
            pInst->opcode = inst & 0xFE00;
            pInst->reg = (inst & 0x01C0) >> 6;
            pInst->srcMode = (inst & 0x0038) >> 3;
            pInst->src = (inst & 0x0007) >> 0;
            pInst->offset = -((int)((inst & 0x003F) >> 0));
        }
        else    // 1111 xxxx xxxx xxxx
        {
            DEBUG("UNKNOWN INSTRUCTION: 0%06o", inst);
            assert(false);
        }
    }
    else    // xIII xxxx xxxx xxxx; III != 0 && III != 7
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

        //DEBUG("Decoder: Double-operand instruction");

        pInst->type = _instt_double_op;
        pInst->opcode = inst & 0xF000;
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

static const char* _formatInstructionOperand(enum _addressingMode mode, int reg, cpu_addr *pc)
{
    static char res[64] = "";

    int pos = 0;

    if(reg == 7)
    {
        switch(mode)
        {
            default:
                break;

            case _auto_inc: // PC Immediate
            {
                cpu_word data = mem_read(*pc += 2, cpu_space_I, PSW_GET_CUR_MODE(cpu.PSW));
                if(data & MEM_HAS_ERR)
                    pos += sprintf(res + pos, "#?[MEM_FAULT]");
                else
                    pos += sprintf(res + pos, "#%06o", data);

                return res;
            }

            case _auto_inc_deferred: // PC Absolute
            {
                cpu_word data = mem_read(*pc += 2, cpu_space_I, PSW_GET_CUR_MODE(cpu.PSW));
                if(data & MEM_HAS_ERR)
                    pos += sprintf(res + pos, "@#?[MEM_FAULT]");
                else
                    pos += sprintf(res + pos, "@#%06o", data);

                return res;
            }
        }
    }

    switch(mode)
    {
        default:
            break;

        case _index:
        {
            cpu_word data = mem_read(*pc += 2, cpu_space_I, PSW_GET_CUR_MODE(cpu.PSW));
            if(data & MEM_HAS_ERR)
                pos += sprintf(res + pos, "X[MEM_FAULT](%s)", cpu.arrRegNameLUT[reg]);
            else
                pos += sprintf(res + pos, "#%06o(%s)", data, cpu.arrRegNameLUT[reg]);

            return res;
        }

        case _index_deferred:
        {
            cpu_word data = mem_read(*pc += 2, cpu_space_I, PSW_GET_CUR_MODE(cpu.PSW));
            if(data & MEM_HAS_ERR)
                pos += sprintf(res + pos, "@X[MEM_FAULT](%s)", cpu.arrRegNameLUT[reg]);
            else
                pos += sprintf(res + pos, "@#%06o(%s)", data, cpu.arrRegNameLUT[reg]);

            return res;
        }
    }

    sprintf(res, "%s%s%s",
        cpu.arrAddrModePrefixLUT[mode],
        cpu.arrRegNameLUT[reg],
        cpu.arrAddrModeSuffixLUT[mode]);

    return res;
}

static const char* _formatInstruction(cpu_addr pc, cpu_word instWord, const struct _instruction* pInst)
{
    static char res[256] = "";

    int pos = sprintf(res, "%06o: %06o\t%s", pc, instWord, cpu.arrOpcodeNameLUT[pInst->opcode]);

    switch(pInst->type)
    {
        default:
        case _instt_unknown:
            pos += sprintf(res + pos, " ?");
            break;

        case _instt_single_op:
            pos += sprintf(res + pos, " %s", _formatInstructionOperand(pInst->dstMode, pInst->dst, &pc));
            break;

        case _instt_double_op:
            pos += sprintf(res + pos, " %s", _formatInstructionOperand(pInst->srcMode, pInst->src, &pc));
            pos += sprintf(res + pos, ", %s", _formatInstructionOperand(pInst->dstMode, pInst->dst, &pc));
            break;

        case _instt_branch:
            pos += sprintf(res + pos, " .%+d", pInst->offset);
            break;

        case _instt_rts:
            pos += sprintf(res + pos, " %s", cpu.arrRegNameLUT[pInst->dst]);
            break;

        case _instt_spl:
            pos += sprintf(res + pos, " %o", pInst->dst);
            break;

        case _instt_double_op_reg_src:
            pos += sprintf(res + pos, " %s", _formatInstructionOperand(pInst->srcMode, pInst->src, &pc));
            pos += sprintf(res + pos, ", %s", cpu.arrRegNameLUT[pInst->reg]);
            break;

        case _instt_system:
            break;
    }

    return res;
}

static bool _makeOperandAddress(enum _addressingMode mode, int reg, bool bByte, operand_addr *pAddr)
{
    assert(mode >= _reg && mode <= _index_deferred);
    assert(reg >= 0 && reg < 8);

    cpu_word *pRn = cpu.GPR + reg;
    int nRnDiff = 0;
    cpu_word addr;

    *pAddr = 0;

    switch(mode)
    {
        default:
        case _reg:
            *pAddr |= OPERAND_TYPE_REG;
            addr = reg;
            break;

        case _reg_deferred:
            addr = *pRn;
            break;

        case _auto_inc:
            addr = *pRn;
            *pRn += nRnDiff = 1 + (!bByte || reg == 6 || reg == 7);
            break;

        case _auto_inc_deferred:
            if(!_read(*pRn, false, cpu_space_D, PSW_GET_CUR_MODE(cpu.PSW), &addr))
                return false;

            *pRn += nRnDiff = 2;
            break;

        case _auto_dec:
            *pRn += nRnDiff = -(1 + (!bByte || reg == 6 || reg == 7));
            addr = *pRn;
            break;

        case _auto_dec_deferred:
            nRnDiff = -2;

            if(!_read(*pRn, false, cpu_space_D, PSW_GET_CUR_MODE(cpu.PSW), &addr))
                return false;

            *pRn += nRnDiff;
            break;

        case _index:
        {
            // Make sure that index was fetched before reading register value
            // This is important in case if register itself is PC
            cpu_word X;
            if(!(nRnDiff = _fetchPC(&X)))
                return false;

            addr = *pRn + X;
            break;
        }

        case _index_deferred:
        {
            // Make sure that index was fetched before reading register value
            // This is important in case if register itself is PC
            cpu_word X;
            if(!(nRnDiff = _fetchPC(&X)))
                return false;

            if(!_read(*pRn + X, false, cpu_space_D, PSW_GET_CUR_MODE(cpu.PSW), &addr))
                return false;
            break;
        }
    }

    *pAddr |= addr;
    if(bByte)
        *pAddr |= OPERAND_TYPE_BYTE;

    mem_updateMMR1(reg, nRnDiff);

    return true;
}

static bool _load(enum _addressingMode mode, int reg, bool bByte, operand_addr* pAddr, cpu_word* data)
{
    if(!_makeOperandAddress(mode, reg, bByte, pAddr))
        return false;

    if(*pAddr & OPERAND_TYPE_REG)
    {
        *data = _signExtend(cpu.GPR[reg], bByte);
        return true;
    }

    return _read(*pAddr, bByte, cpu_space_D, PSW_GET_CUR_MODE(cpu.PSW), data);
}

static bool _store(operand_addr addr, cpu_word data)
{
    if(addr & OPERAND_TYPE_REG)
    {
        cpu.GPR[addr & 7] = data;
        return true;
    }

    return _write(addr, addr & OPERAND_TYPE_BYTE, data, cpu_space_D, PSW_GET_CUR_MODE(cpu.PSW));
}

static bool _push(cpu_word w)
{
    operand_addr addr;
    return _makeOperandAddress(_auto_dec, 6, false, &addr) && _store(addr, w);
}

static bool _pop(cpu_word* w)
{
    operand_addr addr;
    return _load(_auto_inc, 6, false, &addr, w);
}

static void _setFlags(bool n, bool z, bool v, bool c)
{
    cpu.PSW = (cpu.PSW & ~0x000F) | (n << 3) | (z << 2) | (v << 1) | (c << 0);
}

static void _setPSW(cpu_word psw)
{
    cpu.lastSP[PSW_GET_CUR_MODE(cpu.PSW)] = cpu.GPR[6];
    cpu_word lastPC = cpu.GPR[7];

    cpu.GPR = cpu.regSet[PSW_GET_REG_SET(psw)];
    cpu.GPR[6] = cpu.lastSP[PSW_GET_CUR_MODE(psw)];
    cpu.GPR[7] = lastPC;

    cpu.PSW = psw;
}

static void _trap(cpu_word vec)
{
    DEBUG("TRAP %u", vec);

    if(!_push(cpu.PSW) || !_push(cpu.GPR[7]))
    {
        // TODO: Trap from trap
        assert(false);
    }

    cpu_word trapPC;
    cpu_word trapPSW;
    if(   !_read(vec, false, cpu_space_D, cpu_mode_Kernel, &trapPC)
       || !_read(vec + 2, false, cpu_space_D, cpu_mode_Kernel, &trapPSW))
    {
        // TODO: Trap from trap
        assert(false);
    }

    _setPSW(PSW_SET_PREV_MODE(trapPSW, PSW_GET_CUR_MODE(cpu.PSW)));
    cpu.GPR[7] = trapPC;
}

static void _initNameLUT(void)
{
    cpu.arrRegNameLUT[0] = "R0";
    cpu.arrRegNameLUT[1] = "R1";
    cpu.arrRegNameLUT[2] = "R2";
    cpu.arrRegNameLUT[3] = "R3";
    cpu.arrRegNameLUT[4] = "R4";
    cpu.arrRegNameLUT[5] = "R5";
    cpu.arrRegNameLUT[6] = "SP";
    cpu.arrRegNameLUT[7] = "PC";

    cpu.arrAddrModePrefixLUT[_reg]                  = "";
    cpu.arrAddrModePrefixLUT[_reg_deferred]         = "(";
    cpu.arrAddrModePrefixLUT[_auto_inc]             = "(";
    cpu.arrAddrModePrefixLUT[_auto_inc_deferred]    = "@(";
    cpu.arrAddrModePrefixLUT[_auto_dec]             = "-(";
    cpu.arrAddrModePrefixLUT[_auto_dec_deferred]    = "@-(";
    cpu.arrAddrModePrefixLUT[_index]                = "X(";
    cpu.arrAddrModePrefixLUT[_index_deferred]       = "@X(";

    cpu.arrAddrModeSuffixLUT[_reg]                  = "";
    cpu.arrAddrModeSuffixLUT[_reg_deferred]         = ")";
    cpu.arrAddrModeSuffixLUT[_auto_inc]             = ")+";
    cpu.arrAddrModeSuffixLUT[_auto_inc_deferred]    = ")+";
    cpu.arrAddrModeSuffixLUT[_auto_dec]             = ")";
    cpu.arrAddrModeSuffixLUT[_auto_dec_deferred]    = ")";
    cpu.arrAddrModeSuffixLUT[_index]                = ")";
    cpu.arrAddrModeSuffixLUT[_index_deferred]       = ")";

    for(size_t i = 0; i < sizeof(cpu.arrOpcodeNameLUT) / sizeof(cpu.arrOpcodeNameLUT[0]); ++i)
        cpu.arrOpcodeNameLUT[i] = "???";

    cpu.arrOpcodeNameLUT[_mov]    = "MOV";
    cpu.arrOpcodeNameLUT[_movb]   = "MOVB";
    cpu.arrOpcodeNameLUT[_cmp]    = "CMP";
    cpu.arrOpcodeNameLUT[_cmpb]   = "CMPB";
    cpu.arrOpcodeNameLUT[_bit]    = "BIT";
    cpu.arrOpcodeNameLUT[_bitb]   = "BITB";
    cpu.arrOpcodeNameLUT[_bic]    = "BIC";
    cpu.arrOpcodeNameLUT[_bicb]   = "BICB";
    cpu.arrOpcodeNameLUT[_bis]    = "BIS";
    cpu.arrOpcodeNameLUT[_bisb]   = "BISB";
    cpu.arrOpcodeNameLUT[_add]    = "ADD";
    cpu.arrOpcodeNameLUT[_sub]    = "SUB";
    cpu.arrOpcodeNameLUT[_mul]    = "MUL";
    cpu.arrOpcodeNameLUT[_div]    = "DIV";
    cpu.arrOpcodeNameLUT[_ash]    = "ASH";
    cpu.arrOpcodeNameLUT[_ashc]   = "ASHC";
    cpu.arrOpcodeNameLUT[_xor]    = "XOR";
    cpu.arrOpcodeNameLUT[_fp]     = "FP";
    cpu.arrOpcodeNameLUT[_sys]    = "SYS";
    cpu.arrOpcodeNameLUT[_sob]    = "SOB";
    cpu.arrOpcodeNameLUT[_swab]   = "SWAB";
    cpu.arrOpcodeNameLUT[_clr]    = "CLR";
    cpu.arrOpcodeNameLUT[_clrb]   = "CLRB";
    cpu.arrOpcodeNameLUT[_com]    = "COM";
    cpu.arrOpcodeNameLUT[_comb]   = "COMB";
    cpu.arrOpcodeNameLUT[_inc]    = "INC";
    cpu.arrOpcodeNameLUT[_incb]   = "INCB";
    cpu.arrOpcodeNameLUT[_dec]    = "DEC";
    cpu.arrOpcodeNameLUT[_decb]   = "DECB";
    cpu.arrOpcodeNameLUT[_neg]    = "NEG";
    cpu.arrOpcodeNameLUT[_negb]   = "NEGB";
    cpu.arrOpcodeNameLUT[_adc]    = "ADC";
    cpu.arrOpcodeNameLUT[_adcb]   = "ADCB";
    cpu.arrOpcodeNameLUT[_sbc]    = "SBC";
    cpu.arrOpcodeNameLUT[_sbcb]   = "SBCB";
    cpu.arrOpcodeNameLUT[_tst]    = "TST";
    cpu.arrOpcodeNameLUT[_tstb]   = "TSTB";
    cpu.arrOpcodeNameLUT[_ror]    = "ROR";
    cpu.arrOpcodeNameLUT[_rorb]   = "RORB";
    cpu.arrOpcodeNameLUT[_rol]    = "ROL";
    cpu.arrOpcodeNameLUT[_rolb]   = "ROLB";
    cpu.arrOpcodeNameLUT[_asr]    = "ASR";
    cpu.arrOpcodeNameLUT[_asrb]   = "ASRB";
    cpu.arrOpcodeNameLUT[_asl]    = "ASL";
    cpu.arrOpcodeNameLUT[_aslb]   = "ASLB";
    cpu.arrOpcodeNameLUT[_mark]   = "MARK";
    cpu.arrOpcodeNameLUT[_mtps]   = "MTPS";
    cpu.arrOpcodeNameLUT[_mfpi]   = "MFPI";
    cpu.arrOpcodeNameLUT[_mfpd]   = "MFPD";
    cpu.arrOpcodeNameLUT[_mtpi]   = "MTPI";
    cpu.arrOpcodeNameLUT[_mtpd]   = "MTPD";
    cpu.arrOpcodeNameLUT[_sxt]    = "SXT";
    cpu.arrOpcodeNameLUT[_mfps]   = "MFPS";
    cpu.arrOpcodeNameLUT[_br]     = "BR";
    cpu.arrOpcodeNameLUT[_bne]    = "BNE";
    cpu.arrOpcodeNameLUT[_beq]    = "BEQ";
    cpu.arrOpcodeNameLUT[_bge]    = "BGE";
    cpu.arrOpcodeNameLUT[_blt]    = "BLT";
    cpu.arrOpcodeNameLUT[_bgt]    = "BGT";
    cpu.arrOpcodeNameLUT[_ble]    = "BLE";
    cpu.arrOpcodeNameLUT[_bpl]    = "BPL";
    cpu.arrOpcodeNameLUT[_bmi]    = "BMI";
    cpu.arrOpcodeNameLUT[_bhi]    = "BHI";
    cpu.arrOpcodeNameLUT[_blos]   = "BLOS";
    cpu.arrOpcodeNameLUT[_bvc]    = "BVC";
    cpu.arrOpcodeNameLUT[_bvs]    = "BVS";
    cpu.arrOpcodeNameLUT[_bcc]    = "BCC/BHIS";
    cpu.arrOpcodeNameLUT[_bcs]    = "BCS/BLO";
    cpu.arrOpcodeNameLUT[_jmp]    = "JMP";
    cpu.arrOpcodeNameLUT[_jsr]    = "JSR";
    cpu.arrOpcodeNameLUT[_rts]    = "RTS";
    cpu.arrOpcodeNameLUT[_reset]  = "RESET";
    cpu.arrOpcodeNameLUT[_spl]    = "SPL";
    cpu.arrOpcodeNameLUT[_rti]    = "RTI";
}

static cpu_word _cpuDeviceRead(un_addr addr, void* arg)
{
    (void)arg;

    assert(!(addr & 1));

    switch(addr)
    {
        case 0777776:   // PS
            //DEBUG("CPU: PS RD");
            return cpu.PSW;

        case 0777570:   // Console Switch & Display Register
            return 0;
    }

    assert(false);
    return 0;
}

static void _cpuDeviceWrite(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    assert(!(addr & 1));

    switch(addr)
    {
        default:
            assert(false);
            break;

        case 0777776:   // PS
            //DEBUG("CPU: PS WR");
            _setPSW(data);
            break;

        case 0777570:   // Console Switch & Display Register
            DEBUG("DISPLAY: 0x%04X", data);
            break;
    }
}

bool cpu_init(cpu_word R7)
{
    memset(&cpu, 0, sizeof(cpu));

    cpu.GPR = cpu.regSet[CPU_MODE_KERNEL];
    _setPSW(0);

    _initNameLUT();

    dev_io_info ioMap[] = {
        { 0777776, 0777776, &_cpuDeviceRead, &_cpuDeviceWrite, NULL },
        { 0777570, 0777570, &_cpuDeviceRead, &_cpuDeviceWrite, NULL },
        { 0 }
    };
    if(!(cpu.device = dev_initDevice(ioMap, 0, NULL, NULL, NULL)))
        return false;

    cpu.GPR[7] = R7;

    return true;
}

void cpu_destroy(void)
{
    dev_deregisterDevice(cpu.device);
    dev_destroyDevice(cpu.device);
    cpu.device = NULL;
}

device_handle cpu_getHandle(void)
{
    return cpu.device;
}

void cpu_run(void)
{
    // TODO: Debug
    //if(cpu.GPR[7] == 0137176)
    {
    //    cpu.bDisassemblyOutput = true;
    }

    // TODO: Check for pre-existing traps

    // MMR1 records any auto increment/decrement of the general purpose
    // registers, including explicit references through the PC. MMR1 is
    // cleared at the beginning of each instruction fetch. Whenever a general
    // purpose register is either autoincremented or autodecremented, the
    // register number and the amount by which the register was modified
    // (in 2's complement notation) is written into MMR1.
    mem_resetMMR1();

    // TODO: Update MMR2 on traps and interrupts also
    // MMR2 is loaded with the 16-bit Virtual Address (VA) at the beginning
    // of each instruction fetch, or with the address Trap Vector at the
    // beginning of an interrupt, T Bit trap, Parity, Odd Address, and Timeout
    //aborts and parity traps. Note that MMR2 does not get the Trap Vector
    //on EMT, TRAP, BPT and lOT instructions.
    mem_updateMMR2(cpu.GPR[7]);

    cpu_word instWord;
    if(!_fetchPC(&instWord))
        return;

    //DEBUG("Instruction fetch: 0%06o: 0%06o", cpu.GPR[7] - 2, instWord);

    struct _instruction inst = {0};
    _decode(instWord, &inst);

    if(cpu.bDisassemblyOutput)
    {
        DEBUG("%s", _formatInstruction(cpu.GPR[7] - 2, instWord, &inst));

        // TODO: Debug
        //int dbg = 0;
    }

    cpu_word srcVal = 0;
    operand_addr srcAddr = 0;

    cpu_word dstVal = 0;
    operand_addr dstAddr = 0;

    bool byteFlag = inst.opcode & 0x8000;

    switch(inst.opcode)
    {
        case _mov:
        case _movb:
        {
            if(   _load(inst.srcMode, inst.src, byteFlag, &srcAddr, &srcVal)
               && _makeOperandAddress(inst.dstMode, inst.dst, byteFlag, &dstAddr)
               && _store(dstAddr, srcVal))
            {
                _setFlags(srcVal & 0x8000, !srcVal, 0, PSW_GET_C(cpu.PSW));
            }
            break;
        }

        case _cmp:
        case _cmpb:
        {
            if(   _load(inst.srcMode, inst.src, byteFlag, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                int32_t res = srcVal - dstVal;
                _setFlags(res < 0, !res, CALC_V(srcVal, dstVal, res), res & 0x10000);
            }
            break;
        }

        case _bit:
        case _bitb:
        {
            if(   _load(inst.srcMode, inst.src, byteFlag, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                dstVal &= srcVal;
                _setFlags(dstVal & 0x8000, !dstVal, 0, PSW_GET_C(cpu.PSW));
            }
            break;
        }

        case _bic:
        case _bicb:
        {
            if(   _load(inst.srcMode, inst.src, byteFlag, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                dstVal &= ~srcVal;
                if(_store(dstAddr, dstVal))
                    _setFlags(dstVal & 0x8000, !dstVal, 0, PSW_GET_C(cpu.PSW));
            }
            break;
        }

        case _bis:
        case _bisb:
        {
            if(   _load(inst.srcMode, inst.src, byteFlag, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                dstVal |= srcVal;
                if(_store(dstAddr, dstVal))
                    _setFlags(dstVal & 0x8000, !dstVal, 0, PSW_GET_C(cpu.PSW));
            }
            break;
        }

        case _add:
        {
            if(   _load(inst.srcMode, inst.src, false, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, false, &dstAddr, &dstVal))
            {
                uint32_t res = srcVal + dstVal;
                if(_store(dstAddr, res))
                    _setFlags(res & 0x8000, !res, CALC_V(srcVal, dstVal, res), res & 0x10000);
            }
            break;
        }

        case _sub:
        {
            if(   _load(inst.srcMode, inst.src, false, &srcAddr, &srcVal)
               && _load(inst.dstMode, inst.dst, false, &dstAddr, &dstVal))
            {
                uint32_t res = dstVal - srcVal;
                if(_store(dstAddr, res))
                    _setFlags(res & 0x8000, !res, CALC_V(srcVal, dstVal, res), res & 0x10000);
            }
            break;
        }

        case _mul:
        {
            if(!_load(inst.srcMode, inst.src, false, &srcAddr, &srcVal))
                break;
            dstVal = cpu.GPR[inst.reg];

            // Convert to signed integer
            int32_t src = ((int32_t)(srcVal & 0x7FFF)) - ((int32_t)(srcVal & 0x8000));
            int32_t dst = ((int32_t)(dstVal & 0x7FFF)) - ((int32_t)(dstVal & 0x8000));

            int32_t mul = src * dst;
            cpu.GPR[inst.reg] = mul >> 16;
            cpu.GPR[inst.reg | 1] = mul;

            _setFlags(mul & 0x80000000, !mul, 0, mul < -32768 || mul >= 32768);
            break;
        }

        case _div:
        {
            if(!_load(inst.srcMode, inst.src, false, &srcAddr, &srcVal))
                break;

            if(srcVal != 0)
            {
                // Convert to signed integer
                int32_t den = ((int32_t)(srcVal & 0x7FFF)) - ((int32_t)(srcVal & 0x8000));

                // Assemble 32 bit value
                uint32_t arg = cpu.GPR[inst.reg | 1] | (((uint32_t)cpu.GPR[inst.reg]) << 16);
                // Convert to signed integer
                int32_t num = ((int32_t)(arg & 0x7FFFFFFF)) - ((int32_t)(arg & 0x80000000ULL));

                int32_t quot = num / den;
                int32_t rem = num % den;
                if(quot < 0x8000 && quot > -0x10000)
                {
                    cpu.GPR[inst.reg] = quot & 0xFFFF;
                    cpu.GPR[inst.reg | 1] = rem & 0xFFFF;

                    _setFlags(quot < 0, !quot, 0, 0);
                }
                else
                    _setFlags(0, 0, 1, 0);  // Result is unrepresentable in 16 bit register
            }
            else
                _setFlags(0, 0, 1, 1);  // Division by 0
            break;
        }

        case _ash:
        {
            if(!_load(inst.srcMode, inst.src, false, &srcAddr, &srcVal))
                break;

            dstVal = cpu.GPR[inst.reg];

            // The shift bits count ranges from -32 (right shift)
            // to +31 (left shift). Choose 64bit uints here to
            // avoid undefined behavior while shifting.
            uint64_t res = dstVal;
            bool c = PSW_GET_C(cpu.PSW);

            if(srcVal & 0x0020)
            {
                // Negative - right shift
                int n = 0x0040 - (srcVal & 0x003F);
                res >>= n;
                if(dstVal & 0x8000)
                    res |= ~(0x7FFFULL >> n);
                c = dstVal & (1ULL << (n - 1));
            }
            else if(srcVal & 0x003F)
            {
                // Positive - right shift
                int n = srcVal & 0x003F;
                res <<= n;
                c = dstVal & (0x8000ULL >> (n - 1));
            }

            cpu.GPR[inst.reg] = res;
            _setFlags(res & 0x8000, !(res & 0xFFFF), (res ^ dstVal) & 0x8000, c);
            break;
        }

        case _ashc: assert(false);

        case _xor: assert(false);

        case _fp: assert(false);

        case _sys: assert(false);

        case _sob:
        {
            --cpu.GPR[inst.reg];
            BRANCH_IF(cpu.GPR[inst.reg], inst.offset);
            break;
        }

        case _swab:
        {
            if(_load(inst.dstMode, inst.dst, false, &dstAddr, &dstVal))
            {
                uint32_t res = (dstVal << 8) | (dstVal >> 8);
                if(_store(dstAddr, res))
                    _setFlags(res & 0x0080, !(res & 0xFF), 0, 0);
            }
            break;
        }

        case _clr:
        case _clrb:
        {
            if(    _makeOperandAddress(inst.dstMode, inst.dst, byteFlag, &dstAddr)
                && _store(dstAddr, 0))
            {
                _setFlags(0, 1, 0, 0);
            }
            break;
        }

        case _com: assert(false);
        case _comb: assert(false);

        case _inc:
        case _incb:
        {
            if(_load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                ++dstVal;
                if(_store(dstAddr, dstVal))
                    _setFlags(dstVal & 0x8000, !dstVal, dstVal == 0x8000, PSW_GET_C(cpu.PSW));
            }
            break;
        }

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
            if(_load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
                _setFlags(dstVal & 0x8000, !dstVal, 0, 0);
            break;
        }

        case _ror: assert(false);
        case _rorb: assert(false);

        case _rol: assert(false);
        case _rolb: assert(false);

        case _asr: assert(false);
        case _asrb: assert(false);

        case _asl:
        case _aslb:
        {
            if(_load(inst.dstMode, inst.dst, byteFlag, &dstAddr, &dstVal))
            {
                uint32_t res = ((uint32_t)dstVal) << 1;
                if(_store(dstAddr, res))
                {
                    bool n = res & 0x8000;
                    bool c = res & 0x10000;
                    _setFlags(n, !res, n != c, c);
                }
            }
            break;
        }

        case _mark: assert(false);

        case _mtps: assert(false);

        case _mfpi:
        {
            if(!_makeOperandAddress(inst.dstMode, inst.dst, false, &dstAddr))
                break;

            if(dstAddr & OPERAND_TYPE_REG)
                dstVal = cpu.GPR[inst.dst];
            else if(!_read(dstAddr, false, cpu_space_I, PSW_GET_PREV_MODE(cpu.PSW), &dstVal))
                break;

            if(_push(dstVal))
                _setFlags(dstVal & 0x8000, !dstVal, 0, PSW_GET_C(cpu.PSW));
            break;
        }

        case _mfpd: assert(false);

        case _mtpi:
        {
            if(   !_makeOperandAddress(inst.dstMode, inst.dst, false, &dstAddr)
               || !_pop(&dstVal))
            {
                break;
            }

            if(dstAddr & OPERAND_TYPE_REG)
                cpu.GPR[inst.dst] = dstVal;
            else if(!_write(dstAddr, false, dstVal, cpu_space_I, PSW_GET_PREV_MODE(cpu.PSW)))
                break;

            _setFlags(dstVal & 0x8000, !dstVal, 0, PSW_GET_C(cpu.PSW));
            break;
        }

        case _mtpd: assert(false);

        case _sxt: assert(false);

        case _mfps: assert(false);

        case _br:   BRANCH_IF(true,                                                              inst.offset); break;
        case _bne:  BRANCH_IF(!PSW_GET_Z(cpu.PSW),                                               inst.offset); break;
        case _beq:  BRANCH_IF(PSW_GET_Z(cpu.PSW),                                                inst.offset); break;
        case _bge:  BRANCH_IF(PSW_GET_N(cpu.PSW) == PSW_GET_V(cpu.PSW),                          inst.offset); break;
        case _blt:  BRANCH_IF(PSW_GET_N(cpu.PSW) != PSW_GET_V(cpu.PSW),                          inst.offset); break;
        case _bgt:  BRANCH_IF(!PSW_GET_Z(cpu.PSW) && (PSW_GET_N(cpu.PSW) == PSW_GET_V(cpu.PSW)), inst.offset); break;
        case _ble:  BRANCH_IF(!PSW_GET_Z(cpu.PSW) && (PSW_GET_N(cpu.PSW) != PSW_GET_V(cpu.PSW)), inst.offset); break;
        case _bpl:  BRANCH_IF(!PSW_GET_N(cpu.PSW),                                               inst.offset); break;
        case _bmi:  BRANCH_IF(PSW_GET_N(cpu.PSW),                                                inst.offset); break;
        case _bhi:  BRANCH_IF(!(PSW_GET_C(cpu.PSW) || PSW_GET_Z(cpu.PSW)),                       inst.offset); break;
        case _blos: BRANCH_IF(PSW_GET_C(cpu.PSW) || PSW_GET_Z(cpu.PSW),                          inst.offset); break;
        case _bvc:  BRANCH_IF(!PSW_GET_V(cpu.PSW),                                               inst.offset); break;
        case _bvs:  BRANCH_IF(PSW_GET_V(cpu.PSW),                                                inst.offset); break;
        case _bcc:  BRANCH_IF(!PSW_GET_C(cpu.PSW),                                               inst.offset); break;
        case _bcs:  BRANCH_IF(PSW_GET_C(cpu.PSW),                                                inst.offset); break;

        case _jmp:
        {
            if(_makeOperandAddress(inst.dstMode, inst.dst, byteFlag, &dstAddr))
            {
                if(dstAddr & OPERAND_TYPE_REG)
                {
                    // TODO: Illegal instruction trap
                    assert(false);
                }
                else
                    cpu.GPR[7] = dstAddr;
            }
            break;
        }

        case _jsr:
        {
            if(_makeOperandAddress(inst.dstMode, inst.dst, byteFlag, &dstAddr))
            {
                if(dstAddr & OPERAND_TYPE_REG)
                {
                    // TODO: Illegal instruction trap
                    assert(false);
                }
                else if(_push(cpu.GPR[inst.src]))
                {
                    cpu.GPR[inst.src] = cpu.GPR[7];
                    cpu.GPR[7] = dstAddr;
                }
            }
            break;
        }

        case _rts:
        {
            cpu.GPR[7] = cpu.GPR[inst.dst];
            _pop(cpu.GPR + inst.dst);
            break;
        }

        case _reset:
        {
            if(PSW_GET_CUR_MODE(cpu.PSW) == CPU_MODE_KERNEL)
            {
                mem_mmu_reset();
                dev_reset();
            }
            break;
        }

        case _spl:
        {
            if(PSW_GET_CUR_MODE(cpu.PSW) == CPU_MODE_KERNEL)
                _setPSW(PSW_SET_PRIORITY(cpu.PSW, inst.dst));
            break;
        }

        case _rti:
        {
            cpu_word PC;
            cpu_word psw;
            if(_pop(&PC) && _pop(&psw))
            {
                psw &= PSW_MASK;
                if(PSW_GET_CUR_MODE(cpu.PSW) == CPU_MODE_KERNEL)
                {
                    // When executed in Supervisor mode, the current and previous
                    // mode bits in the restored PS cannot be Kernel. When executed in
                    // User mode, the current and previous mode bits in the restored PS
                    // can only be User. RTI cannot clear PS<11> if it was already set.
                    // Apparently priority level is also not allowed to be lowered...
                    psw = (psw & 0xF81F) | (cpu.PSW & 0xF8E0);
                }

                cpu.GPR[7] = PC;
                _setPSW(psw);

                // TODO: T bit
            }
            break;
        }

        default:
            DEBUG("UNIMPLEMENTED INSTRUCTION: 0%06o: 0%06o", cpu.GPR[7] - 2, instWord);
            assert(false);
            break;
    }
}
