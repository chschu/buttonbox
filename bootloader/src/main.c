#include <inttypes.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#include <sync_serial.h>

// supported block size that can be read or written by the client at once
// must be an integer multiple of SPM_PAGESIZE and fit inside 16 bits
#define BLOCKSIZE SPM_PAGESIZE

void block_read(uint16_t size,uint8_t mem_type, uint16_t *address) {
    switch (mem_type) {
    case 'E': // EEPROM
        while (size--) {
            // setup EEPROM address
            EEARL = *address;
            EEARH = (*address) >> 8;

            // auto-increment
            (*address)++;

            // read
            EECR |= _BV(EERE);

            // send EEPROM byte
            sync_serial_putc(EEDR);
        }
        break;

    case 'F': // flash
        // wait for previous write/erase to complete
        boot_spm_busy_wait();

        // enable RWW before reading flash
        boot_rww_enable();

        while (size) {
            // send two flash bytes (address is given in words here)
            sync_serial_putc(pgm_read_byte((*address) << 1));
            sync_serial_putc(pgm_read_byte(((*address) << 1) | 1));

            // auto-increment
            (*address)++;

            size -= 2;
        }
    }
}

void block_load(uint16_t size, uint8_t mem_type, uint16_t *address) {
    uint8_t buffer[BLOCKSIZE];
    uint16_t data;
    uint16_t temp_address;

    switch (mem_type) {
    case 'E': // EEPROM
        // read into fast buffer first (EEPROM is slow)
        for (temp_address = 0; temp_address < size; temp_address++) {
            buffer[temp_address] = sync_serial_getc();
        }

        // copy to EEPROM
        for (temp_address = 0; temp_address < size; temp_address++) {
            // setup EEPROM address
            EEARL = *address; 
            EEARH = (*address) >> 8;
            
            // get byte from buffer and write it
            EEDR = buffer[temp_address];
            EECR |= _BV(EEMPE);
            EECR |= _BV(EEPE);
            while (EECR & _BV(EEPE));

            // auto-increment
            (*address)++;
        }

        // OK
        sync_serial_putc('\r');
        break;
    
    case 'F': // flash
        // keep initial address for final page write
        temp_address = *address;

        // wait for previous write/erase to complete
        boot_spm_busy_wait();

        // fill page from received words
        while (size) { 
            // create word from received bytes, LSB first
            data = sync_serial_getc();
            data |= sync_serial_getc() << 8;

            // fill page (address is given in words here)
            boot_page_fill((*address) << 1, data);

            // auto-increment
            (*address)++;
            size -= 2;
        }

        // write page (address is given in words here)
        boot_page_write(temp_address << 1);

        // OK
        sync_serial_putc('\r');
        break;

    default: // invalid memory type
        // not OK
        sync_serial_putc('?');
        break;
    }
}

int main() {
    uint16_t address;
    uint16_t temp_int;
    uint8_t temp_byte;

    // set up function pointer to RESET vector
    void (*reset)() = 0x0000; 

    // enable pull-up on PB3 (MOSI)
    PORTB |= _BV(PORTB3);

    // enter bootloader iff MOSI is pulled low
    if (PINB & _BV(PINB3)) {
        // jump to application
        reset();
    }

    // initialize synchronous serial communication
    sync_serial_init();

    for (;;) {
        // read command byte
        switch (sync_serial_getc()) {
        case 'a': // check auto-increment status
            // enabled
            sync_serial_putc('Y');
            break;

        case 'A': // set address (in words!)
            // receive address, MSB first
            address = (sync_serial_getc() << 8) | sync_serial_getc();

            // OK
            sync_serial_putc('\r');
            break;

        case 'e': // chip erase (flash only)
            for (address = 0; address < BOOT_LOADER_START; address += SPM_PAGESIZE) {
                // wait for previous write/erase to complete
                boot_spm_busy_wait();

                // erase the page
                boot_page_erase(address);
            }

            // OK
            sync_serial_putc('\r');
            break;

        case 'b': // check block load support
            // supported
            sync_serial_putc('Y');

            // result must be an integer multiple of SPM_PAGESIZE, two bytes, MSB first
            sync_serial_putc(BLOCKSIZE >> 8);
            sync_serial_putc(BLOCKSIZE & 0xff);
            break;

        case 'B': // start block load
            // receive block size, MSB first
            temp_int = (sync_serial_getc() << 8) | sync_serial_getc();

            // get memtype (E or F)
            temp_byte = sync_serial_getc();

            // load block, sending result
            block_load(temp_int, temp_byte, &address);
            break;

        case 'g': // start block read
            // receive block size, MSB first
            temp_int = (sync_serial_getc() << 8) | sync_serial_getc();

            // get memtype (E or F)
            temp_byte = sync_serial_getc(); // Get memtype

            // read block, sending bytes
            block_read(temp_int, temp_byte, &address);
            break;

        case 'R': // read program memory
            // enable RWW before reading flash
            boot_rww_enable();

            // send high and low byte of flash word
            sync_serial_putc(pgm_read_byte((address << 1) | 1));
            sync_serial_putc(pgm_read_byte((address << 1)));

            // auto-increment
            address++;
            break;

        case 'c': // write program memory, low byte (must be used directly before 'C')
            // store low byte temporarily
            temp_int = sync_serial_getc();

            // OK
            sync_serial_putc('\r');
            break;

        case 'C': // write program memory, high byte (must be used directly after 'c')
            // combine high and low byte
            temp_int |= sync_serial_getc() << 8;

            // wait for previous write/erase to complete
            boot_spm_busy_wait();

            // convert to byte-address and fill
            boot_page_fill(address << 1, temp_int);
            
            // auto-increment
            address++;

            // OK
            sync_serial_putc('\r');
            break;
    
        case 'm': // write page
            // block boot loader writes
            if (address >= (BOOT_LOADER_START >> 1)) {
                // not OK
                sync_serial_putc('?');
            } else {
                // wait for previous write/erase to complete
                boot_spm_busy_wait();

                // convert to byte-address and write
                boot_page_write(address << 1);

                // OK
                sync_serial_putc('\r');
            }
            break;

        case 'D': // write EEPROM
            // setup EEPROM address
            EEARL = address;
            EEARH = address >> 8;

            // get and write byte
            EEDR = sync_serial_getc();
            EECR |= _BV(EEMPE);
            EECR |= _BV(EEPE);
            while (EECR & _BV(EEPE));
                
            // auto-increment
            address++;

            // OK
            sync_serial_putc('\r');
            break;

        case 'd': // read EEPROM
            // setup EEPROM address
            EEARL = address;
            EEARH = address >> 8;

            // prepare read
            EECR |= _BV(EERE);

            // read byte and send it
            sync_serial_putc(EEDR);

            // auto-increment
            address++;
            break;

        case 'l': // write lock bits
            // not supported because it's not really useful

            // not OK
            sync_serial_putc('?');
            break;

        case 'r': // read lock bits
            sync_serial_putc(boot_lock_fuse_bits_get(GET_LOCK_BITS));
            break;

        case 'F': // read low fuse bits
            sync_serial_putc(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
            break;

        case 'N': // read high fuse bits
            sync_serial_putc(boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS));
            break;

        case 'Q': // read extended fuse bits
            sync_serial_putc(boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS));
            break;

        case 'P': // enter programming mode
        case 'L': // leave programming mode
            // OK
            sync_serial_putc('\r');
            break;

        case 'E': // exit boot loader
            // OK
            sync_serial_putc('\r');

            // wait for previous write/erase to complete
            boot_spm_busy_wait();

            // enable RWW before executing flash
            boot_rww_enable();

            // jump to application
            reset();

            // will never get here
            break;

        case 'p': // get programmer type    
            // serial
            sync_serial_putc('S');
            break;

        case 't': // get supported device types
            // send only list terminator (ATmega328P has no device code)
            sync_serial_putc(0);
            break;

        case 'x': // set LED
        case 'y': // clear LED
        case 'T': // set device type
            // ignore parameter
            sync_serial_getc();

            // OK
            sync_serial_putc('\r');
            break;

        case 'S': // get programmer identifier (7 characters)
            sync_serial_putc('A');
            sync_serial_putc('V');
            sync_serial_putc('R');
            sync_serial_putc('B');
            sync_serial_putc('O');
            sync_serial_putc('O');
            sync_serial_putc('T');
            break;

        case 'V': // get software version
            sync_serial_putc('1');
            sync_serial_putc('5');
            break;

        case 's': // get signature bytes (reverse order)
            sync_serial_putc(0x0F);
            sync_serial_putc(0x95);
            sync_serial_putc(0x1E);
            break;

        case 0x1b: // escape (sync)
            break;

        default: // unknown
            sync_serial_putc('?');
            break;
        }
    }
}
