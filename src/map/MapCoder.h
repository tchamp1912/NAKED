#ifndef MAPCODER
#define MAPCODER

// Responsible for encoding/decoding to/from the struct found in Map.h 
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "Map.h"
#include "QrCode.hpp"

class MapCoder {
    public:
        void* encode(Map map) {
            void* bitstream = new char[406]; // 406 == 101x101 QR code max bytes | TODO don't allocate a hard number
            // Encode grid edge length
            float* grid_edge_length = (float*)bitstream;
            grid_edge_length[0] =  map.gridEdgeLength;
            short* streamPointer = (short*)bitstream;
            streamPointer += 2;
            // Encode other fields of the Map
            streamPointer = encodeBlockedCells(streamPointer, map);
            streamPointer = encodeBlockedChunks(streamPointer, map);
            streamPointer = encodeTargets(streamPointer, map);
            streamPointer = encodeQRCodes(streamPointer, map);
            return bitstream;
        }

        void generateQr(void* bitstream) {
            char* stream = (char*)bitstream;
	        const qrcodegen::QrCode::Ecc errCorLvl = qrcodegen::QrCode::Ecc::LOW;  // Error correction level
            const qrcodegen::QrCode qr = qrcodegen::QrCode::encodeText(stream, errCorLvl);
            // Made qr, now save in NetPBM/PBMplus format
            FILE *imageFile = fopen("qr.pgm","wb");
            int pixel, size = qr.getSize();
            fprintf(imageFile, "P5\n");                     // P5 filetype
            fprintf(imageFile, "%d %d\n", size, size);      // Dimensions
            fprintf(imageFile,"255\n");                     // Max pixel
            for(int x = 0; x < size; x++){
                for(int y = 0; y < size; y++){
                    pixel = qr.getModule(x, y) ? 0 : 255;
                    std::cout << (qr.getModule(x, y) ? "##" : "  ");
                    fputc(pixel,imageFile);
                }
                std::cout << std::endl;
            }
            fclose(imageFile);
            //std::cout << qr.toSvgString(4) << std::endl;
    }

        Map decode(void* bitstream) {
            Map map;
            // Decode grid edge length
            map.gridEdgeLength = *(float*)bitstream;
            short* streamPointer = (short*)bitstream;
            streamPointer += 2;
            bool reading = true;
            // Decode other fields of the Map
            streamPointer = decodeBlockedCells(streamPointer, &map);
            streamPointer = decodeBlockedChunks(streamPointer, &map);
            streamPointer = decodeTargets(streamPointer, &map);
            streamPointer = decodeQRCodes(streamPointer, &map);
            return map;
        }

    private:
        short* encodeBlockedCells(short* streamPointer, Map map) {
            short cells = map.blockedCells.size();
            streamPointer[0] = cells;
            streamPointer += 1;
            for (int i = 0; i < map.blockedCells.size(); i++) {
                Cell cell = map.blockedCells[i];
                streamPointer[0] = cell.x;
                streamPointer[1] = cell.y;
                streamPointer += 2;
            }
            return streamPointer;
        }

        short* encodeBlockedChunks(short* streamPointer, Map map) {
            short chunks = map.blockedChunks.size();
            streamPointer[0] = chunks;
            streamPointer += 1;
            for (int i = 0; i < map.blockedChunks.size(); i++) {
                Chunk chunk = map.blockedChunks[i];
                streamPointer[0] = chunk.topLeft.x;
                streamPointer[1] = chunk.topLeft.y;
                streamPointer[2] = chunk.bottomRight.x;
                streamPointer[3] = chunk.bottomRight.y;
                streamPointer += 4;
            }
            return streamPointer;
        }

        short* encodeTargets(short* streamPointer, Map map) {
            short targets = map.targets.size();
            streamPointer[0] = targets;
            streamPointer += 1;
            for (int i = 0; i < map.targets.size(); i++) {
                Target target = map.targets[i];
                streamPointer[0] = target.x;
                streamPointer[1] = target.y;
                streamPointer[2] = target.instruction;
                streamPointer += 3;
            }
            return streamPointer;
        }

        short* encodeQRCodes(short* streamPointer, Map map) {
            short qrCodes = map.qrCodes.size();
            streamPointer[0] = qrCodes;
            streamPointer += 1;
            for (int i = 0; i < map.qrCodes.size(); i++) {
                Cell qrCode = map.qrCodes[i];
                streamPointer[0] = qrCode.x;
                streamPointer[1] = qrCode.y;
                streamPointer += 2;
            }
            return streamPointer;
        }

        short* decodeBlockedCells(short* streamPointer, Map* map) {
            short cells = streamPointer[0];
            streamPointer += 1;
            for (int i = 0; i < cells; i++) {
                Cell cell(streamPointer[0], streamPointer[1]);
                map->blockedCells.push_back(cell);
                streamPointer += 2;
            }
            return streamPointer;
        }

        short* decodeBlockedChunks(short* streamPointer, Map* map) {
            short chunks = streamPointer[0];
            streamPointer += 1;
            for (int i = 0; i < chunks; i++) {
                Cell tl(streamPointer[0], streamPointer[1]);
                Cell br(streamPointer[2], streamPointer[3]);
                Chunk chunk(tl, br);
                map->blockedChunks.push_back(chunk);
                streamPointer += 4;
            }
            return streamPointer;
        }

        short* decodeTargets(short* streamPointer, Map* map) {
            short targets = streamPointer[0];
            streamPointer += 1;
            for (int i = 0; i < targets; i++) {
                Target target(streamPointer[0], streamPointer[1], streamPointer[2]);
                map->targets.push_back(target);
                streamPointer += 3;
            }
            return streamPointer;
        }

        short* decodeQRCodes(short* streamPointer, Map* map) {
            short qrCodes = streamPointer[0];
            streamPointer += 1;
            for (int i = 0; i < qrCodes; i++) {
                Cell qrCode(streamPointer[0], streamPointer[1]);
                map->qrCodes.push_back(qrCode);
                streamPointer += 2;
            }
            return streamPointer;
        }

        /** Utility */
        void printQr(const qrcodegen::QrCode &qr) {
            int border = 4;
            for (int y = -border; y < qr.getSize() + border; y++) {
                for (int x = -border; x < qr.getSize() + border; x++) {
                    std::cout << (qr.getModule(x, y) ? "##" : "  ");
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
};

#endif