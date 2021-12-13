//
// Created by palulukan on 12/11/21.
//

#include "mmu.h"

#include "log.h"

#include <assert.h>
#include <string.h>

#define MMU_MMR0_MMU_EN         (1 << 0)    // Enable Relocation
#define MMU_MMR0_PAGE_NO_MASK   0x000E      // Page Number
#define MMU_MMR0_SET_PAGE_NO(mmr, page) ((mmr) = ((mmr) & ~MMU_MMR0_PAGE_NO_MASK) | (((page) << 1) & MMU_MMR0_PAGE_NO_MASK))
#define MMU_MMR0_PAGE_SPACE     (1 << 4)    // Page Address Space
#define MMU_MMR0_CPU_MODE_MASK  0x0060      // Processor Mode
#define MMU_MMR0_SET_CPU_MODE_NO(mmr, mode) ((mmr) = ((mmr) & ~MMU_MMR0_CPU_MODE_MASK) | (((mode) << 5) & MMU_MMR0_CPU_MODE_MASK))
#define MMU_MMR0_INST_COMPLETED (1 << 7)    // Instruction Completed
#define MMU_MMR0_DST_MODE       (1 << 8)    // Maintenance/Destination Mode
#define MMU_MMR0_TRAP_EN        (1 << 9)    // Enable Memory Management Traps
#define MMU_MMR0_TRAP           (1 << 12)   // Trap - set whenever a Memory Management trap condition occurs
#define MMU_MMR0_ERR_ABRT_RO    (1 << 13)   // Abort - Read Only
#define MMU_MMR0_ERR_ABRT_PL    (1 << 14)   // Abort - Page Length
#define MMU_MMR0_ERR_ABRT_NR    (1 << 15)   // Abort - Non-Resident
#define MMU_MMR0_ERR_MASK   (MMU_MMR0_ERR_ABRT_RO | MMU_MMR0_ERR_ABRT_PL | MMU_MMR0_ERR_ABRT_NR)
#define MMU_MMR0_WR_MASK    (MMU_MMR0_MMU_EN | MMU_MMR0_PAGE_NO_MASK | MMU_MMR0_PAGE_SPACE \
    | MMU_MMR0_CPU_MODE_MASK | MMU_MMR0_DST_MODE | MMU_MMR0_TRAP_EN | MMU_MMR0_TRAP | MMU_MMR0_ERR_MASK)

#define MMU_MMR3_USR_D_EN   (1 << 0)    // Enable User D Space
#define MMU_MMR3_SVR_D_EN   (1 << 1)    // Enable Supervisor D Space
#define MMU_MMR3_KRN_D_EN   (1 << 2)    // Enable Kernel D Space
#define MMU_MMR3_22BIT_MAP  (1 << 4)    // Enable 22-bit mapping
#define MMU_MMR3_UB_MAP_REL (1 << 5)    // Enable UNIBUS Map relocation
#define MMU_MMR3_WR_MASK (MMU_MMR3_USR_D_EN | MMU_MMR3_SVR_D_EN | MMU_MMR3_KRN_D_EN | MMU_MMR3_22BIT_MAP | MMU_MMR3_UB_MAP_REL)

#define PDR_ACF_MASK        0x0007  // Access Control Field (ACF)
                                    // 000  non-resident    abort all accesses
                                    // 001  read-only*      abort on write attempt, memory management trap on read
                                    // 010  read-only       abort on write attempt
                                    // 011  unused          abort all accesses-reserved for future use
                                    // 100  read/write      memory management trap upon completion of a read or write
                                    // 101  read/write*     memory management trap upon completion of a write
                                    // 110  read/write      no system trap/abort action
                                    // 111  unused          abort all accesses-reserved for future use
#define PDR_GET_ACF(pdr)    ((pdr) & PDR_ACF_MASK)
#define PDR_ED              (1 << 3)    // Expansion Direction
#define PDR_W               (1 << 6)    // Access Information Bit W
#define PDR_A               (1 << 7)    // Access Information Bit A
#define PDR_PLF_MASK        0x7F00      // Page Length Field
#define PDR_GET_PLF(pdr)    (((pdr) >> 8) & 0x7F)
#define PDR_WR_MASK         (PDR_PLF_MASK | PDR_ED | PDR_ACF_MASK)

struct
{
    cpu_word MMR[4];

    struct {
        struct {
            cpu_word PAR[8];
            cpu_word PDR[8];
        } page_reg_spaces[_cpu_space_max];
    } page_reg_modes[_cpu_mode_max];
} mmu;

struct _mmuRegGroupInfo
{
    un_addr ioStart;
    un_addr ioEnd;
    cpu_mode m;
    cpu_space s;
};

const static struct _mmuRegGroupInfo mmuGegGroupsTablePDR[6] = {
    // PDR
    { 0777600, 0777616, cpu_mode_User, cpu_space_I },       // UISDR[0-7]
    { 0777620, 0777636, cpu_mode_User, cpu_space_D },       // UDSDR[0-7]

    { 0772200, 0772216, cpu_mode_Supervisor, cpu_space_I }, // SISDR[0-7]
    { 0772220, 0772236, cpu_mode_Supervisor, cpu_space_D }, // SDSDR[0-7]

    { 0772300, 0772316, cpu_mode_Kernel, cpu_space_I},      // KISDR[0-7]
    { 0772320, 0772336, cpu_mode_Kernel, cpu_space_D },     // KDSDR[0-7]
};

const static struct _mmuRegGroupInfo mmuGegGroupsTablePAR[6] = {
    // PAR
    { 0777640, 0777656, cpu_mode_User, cpu_space_I },       // UISAR[0-7]
    { 0777660, 0777676, cpu_mode_User, cpu_space_D },       // UDSAR[0-7]

    { 0772240, 0772256, cpu_mode_Supervisor, cpu_space_I }, // SISAR[0-7]
    { 0772260, 0772276, cpu_mode_Supervisor, cpu_space_D }, // SDSAR[0-7]

    { 0772340, 0772356, cpu_mode_Kernel, cpu_space_I },     // KISAR[0-7]
    { 0772360, 0772376, cpu_mode_Kernel, cpu_space_D },     // KDSAR[0-7]
};

static cpu_word _mmuRegReadPDR(un_addr addr, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

//    DEBUG("MMU: RD PDR%d", nReg);

    return mmu.page_reg_modes[pRG->m].page_reg_spaces[pRG->s].PDR[nReg];
}

static void _mmuRegWritePDR(un_addr addr, cpu_word data, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

//    DEBUG("MMU: WR PDR%d", nReg);

    mmu.page_reg_modes[pRG->m].page_reg_spaces[pRG->s].PDR[nReg] = data & PDR_WR_MASK;
}

static cpu_word _mmuRegReadPAR(un_addr addr, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

//    DEBUG("MMU: RD PAR%d", nReg);

    return mmu.page_reg_modes[pRG->m].page_reg_spaces[pRG->s].PAR[nReg];
}

static void _mmuRegWritePAR(un_addr addr, cpu_word data, void* arg)
{
    struct _mmuRegGroupInfo *pRG = (struct _mmuRegGroupInfo *)arg;

    assert(addr >= pRG->ioStart && addr <= pRG->ioEnd);
    assert(!(addr & 1));

    int nReg = (addr - pRG->ioStart) >> 1;

//    DEBUG("MMU: WR PAR%d", nReg);

    mmu.page_reg_modes[pRG->m].page_reg_spaces[pRG->s].PAR[nReg] = data;
    mmu.page_reg_modes[pRG->m].page_reg_spaces[pRG->s].PDR[nReg] &= ~(PDR_A | PDR_W);
}

static cpu_word _regRead(un_addr addr, void* arg)
{
    (void)arg;

    switch(addr)
    {
        case 0777572:   // MMR0
            DEBUG("MMU: RD MMR0");
            return mmu.MMR[0];

        case 0777574:   // MMR1
            DEBUG("MMU: RD MMR1");
            return mmu.MMR[1];

        case 0777576:   // MMR2
            DEBUG("MMU: RD MMR2");
            return mmu.MMR[2];

        case 0772516:   // MMR3
            DEBUG("MMU: RD MMR3");
            return mmu.MMR[3];

//        case 0777760:   // Lower Size Register
//            return (MEM_SIZE_BYTES >> 6) - 1;
//
//        case 0777762:   // Upper Size Register
//            return 0;
    }

    assert(false);
    return 0;
}

static void _regWrite(un_addr addr, cpu_word data, void* arg)
{
    (void)arg;

    switch(addr)
    {
        default:
            assert(false);
            break;

        case 0777572:   // MMR0
            DEBUG("MMU: WR MMR0");
            assert(!(data & MMU_MMR0_DST_MODE));    // TODO: Add support for maintenance mode
            mmu.MMR[0] = data & MMU_MMR0_WR_MASK;
            break;

        case 0777574:   // MMR1
            DEBUG("MMU: WR MMR1");
            mmu.MMR[1] = data;
            break;

        case 0777576:   // MMR2 is read-only
            DEBUG("MMU: WR MMR2");
            break;

        case 0772516:   // MMR3
            DEBUG("MMU: WR MMR3");
            assert(!(data & MMU_MMR3_22BIT_MAP));   // TODO: Implement
            assert(!(data & MMU_MMR3_UB_MAP_REL));  // TODO: Implement
            mmu.MMR[3] = data & MMU_MMR3_WR_MASK;
            break;

//        case 0777760:   // Lower Size Register
//        case 0777762:   // Upper Size Register
//            break;
    }
}

void mmu_init(void)
{
    mmu_reset();

    // Memory management registers
    mem_registerUnibusIO(0777572, 0777576, &_regRead, &_regWrite, NULL);  // MMR0 - MMR2
    mem_registerUnibusIO(0772516, 0772516, &_regRead, &_regWrite, NULL);  // MMR3
    mem_registerUnibusIO(0777760, 0777762, &_regRead, &_regWrite, NULL);  // Lower and Upper Size Register

    for(size_t i = 0; i < sizeof(mmuGegGroupsTablePDR) / sizeof(mmuGegGroupsTablePDR[0]); ++i)
    {
        struct _mmuRegGroupInfo *pRG = mmuGegGroupsTablePDR + i;
        mem_registerUnibusIO(pRG->ioStart, pRG->ioEnd, &_mmuRegReadPDR, &_mmuRegWritePDR, pRG);
    }

    for(size_t i = 0; i < sizeof(mmuGegGroupsTablePAR) / sizeof(mmuGegGroupsTablePAR[0]); ++i)
    {
        struct _mmuRegGroupInfo *pRG = mmuGegGroupsTablePAR + i;
        mem_registerUnibusIO(pRG->ioStart, pRG->ioEnd, &_mmuRegReadPAR, &_mmuRegWritePAR, pRG);
    }
}

void mmu_reset(void)
{
    memset(&mmu, 0, sizeof(mmu));
    mmu.MMR[0] |= MMU_MMR0_INST_COMPLETED | MMU_MMR0_TRAP_EN;
}

void mmu_updateMMR0(bool bInstCompleted)
{
    mmu.MMR[0] = (mmu.MMR[0] & ~MMU_MMR0_INST_COMPLETED) | (bInstCompleted ? MMU_MMR0_INST_COMPLETED : 0);
}

void mmu_resetMMR1(void)
{
    if(!(mmu.MMR[0] & MMU_MMR0_ERR_MASK))
        mmu.MMR[1] = 0;
}

void mmu_updateMMR1(int reg, int diff)
{
    assert(reg >= 0);
    assert(reg < 8);
    assert(diff >= -16);
    assert(diff < 16);

    if(!(mmu.MMR[0] & MMU_MMR0_ERR_MASK))
    {
        mmu.MMR[1] <<= 8;
        mmu.MMR[1] = ((diff << 3) & 0x00F8) | (reg & 7);
    }
}

void mmu_updateMMR2(cpu_word val)
{
    if(!(mmu.MMR[0] & MMU_MMR0_ERR_MASK))
        mmu.MMR[2] = val;
}

ph_addr mmu_map(cpu_addr addr, cpu_space s, cpu_mode m, bool bWR)
{
    assert(s < _cpu_space_max);
    assert(m < _cpu_mode_max);

    if(!(mmu.MMR[0] & MMU_MMR0_MMU_EN))
    {
        // Memory Management Unit is inoperative and addresses are not
        // relocated or protected.

        // 16-bit mapping
        ph_addr PA = addr;
        if(addr >= MEM_16BIT_PERIPH_PAGE_ADDR)
        {
            PA -= MEM_16BIT_PERIPH_PAGE_ADDR;
            PA += MEM_UNIBUS_PERIPH_PAGE_ADDR + MEM_22BIT_UNIBUS_ADDR;
        }

        return PA;
    }

    if(m == cpu_mode_Invalid)
    {
        // TODO: Trap
        assert(false);
    }

    if(   (m == cpu_mode_Kernel     && !(mmu.MMR[3] & MMU_MMR3_KRN_D_EN))
       || (m == cpu_mode_Supervisor && !(mmu.MMR[3] & MMU_MMR3_SVR_D_EN))
       || (m == cpu_mode_User       && !(mmu.MMR[3] & MMU_MMR3_USR_D_EN)))
    {
        // When D space is disabled, all references use the I space registers
        s = cpu_space_I;
    }

    // TODO: Check MMU mode

    // Select Page Address Register (PAR) and Page Descriptor Register (PDR).
    // Each CPU mode (Kernel, Supervisor and User) has it's own set
    // of PARs and PDRs for each space (Instruction and Data).
    // PAR for current space and mode is selected by
    // Active Page Field (APF) which is 3 most significant
    // bits of the virtual address. The rest 13 bits of the Virtual
    // Address (VA) are called Displacement Field (DF).
    // It consists of Block Number (BN) - higher 7 bits and
    // Displacement In Block (DIB) - lower 6 bits.
    cpu_word PAR = mmu.page_reg_modes[m].page_reg_spaces[s].PAR[(addr >> 13) & 7];
    cpu_word *pPDR = mmu.page_reg_modes[m].page_reg_spaces[s].PDR + ((addr >> 13) & 7);

    cpu_word BN = (addr >> 6) & 0x007F;

    if(((*pPDR & PDR_ED) && (BN < PDR_GET_PLF(*pPDR))) || (!(*pPDR & PDR_ED) && (BN > PDR_GET_PLF(*pPDR))))
    {
        // TODO: Page length violation
        assert(false);
    }

    switch(*pPDR & PDR_ACF_MASK)
    {
        case 0: // non-resident: abort all accesses
        case 3: // unused: abort all accesses - reserved for future use
        case 7: // unused: abort all accesses - reserved for future use
            assert(false);  // TODO: Implement

        case 1: // read-only: abort on write attempt, memory management trap on read
        {
            if(!bWR)
                *pPDR |= PDR_A;

            assert(false);  // TODO: Implement
        }

        case 2: // read-only: abort on write attempt
        {
            if(bWR)
            {
                *pPDR |= PDR_A;

                assert(false);  // TODO: Implement
            }
            break;
        }

        case 4: // read/write: memory management trap upon completion of a read or write
            *pPDR |= PDR_A;
            assert(false);  // TODO: Implement

        case 5: // read/write: memory management trap upon completion of a write
        {
            if(bWR)
                *pPDR |= PDR_A;

            assert(false);  // TODO: Implement
        }

        case 6: // read/write: no system trap/abort action
            break;
    }

    // Page Address Field (PAF) of selected PAR is
    // 12 bits on 11/34A and 11/60 and 16 bits on 11/44 and 11/70.
    // It holds the starting address of the page.
    ph_addr PA = PAR;

    // The Physical Block Number (PBN) is obtained by
    // adding the PAF from PAR to BN from virtual address.
    // PBN will contain our final Physical Address (PA).
    PA += BN;

    // Get final PA by joining 6 bit DIB from VA to PBN.
    PA = (PA << 6) | (addr & 0x003F);

    if(!(mmu.MMR[3] & MMU_MMR3_22BIT_MAP))
    {
        // 18-bit mapping
        if(PA >= MEM_18BIT_PERIPH_PAGE_ADDR)
        {
            PA -= MEM_18BIT_PERIPH_PAGE_ADDR;
            PA += MEM_UNIBUS_PERIPH_PAGE_ADDR + MEM_22BIT_UNIBUS_ADDR;
        }
    }
    else
        assert(false);  // TODO: Implement 22-bit mapping

    return PA;
}

