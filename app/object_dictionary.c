#include "object_dictionary.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// 创建对象字典
ObjectDictionary* OD_Create(void) {
    ObjectDictionary* od = (ObjectDictionary*)malloc(sizeof(ObjectDictionary));
    if (od == NULL) {
        return NULL;
    }

    od->head = NULL;
    od->entryCount = 0;
    return od;
}

// 销毁对象字典
void OD_Destroy(ObjectDictionary* od) {
    if (od == NULL) {
        return;
    }

    // 遍历链表释放所有节点
    OD_Entry* current = od->head;
    while (current != NULL) {
        OD_Entry* next = current->next;
        if (current->data != NULL) {
            free(current->data);
        }
        free(current);
        current = next;
    }

    free(od);
}

// 查找条目
static OD_Entry* OD_FindEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex) {
    if(od == NULL || od->head == NULL)
        return NULL;
        
    OD_Entry* current = od->head;
    while (current != NULL) {
        if (current->index == index && current->subindex == subindex) {
            return current;
        }
		if(current->next != NULL)
			current = current->next;
		else
			return NULL;
    }
    return NULL;
}

// 获取数据类型大小
static size_t OD_GetDataTypeSize(OD_DataType dataType) {
    switch (dataType) {
        case OD_TYPE_BOOLEAN:
        case OD_TYPE_INT8:
        case OD_TYPE_UINT8:
            return 1;
        case OD_TYPE_INT16:
        case OD_TYPE_UINT16:
            return 2;
        case OD_TYPE_INT32:
        case OD_TYPE_UINT32:
        case OD_TYPE_FLOAT32:
            return 4;
        case OD_TYPE_STRING:
            return 256; // 默认字符串长度
        default:
            return 0;
    }
}

// 添加条目
bool OD_AddEntry(ObjectDictionary* od, OD_Entry* entry) {
    if (od == NULL || entry == NULL || OD_FindEntry(od, entry->index, entry->subindex) != NULL) {
        return false;
    }

    // 分配数据空间
    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    entry->data = malloc(dataSize);
    if (entry->data == NULL) {
        return false;
    }

    // 将新节点插入到链表头部
    entry->next = od->head;
    od->head = entry;
    od->entryCount++;

    return true;
}

bool OD_User_AddEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex, OD_DataType dataType, void* data, bool accessRead, bool accessWrite) {
    if (od == NULL) {
        return false;
    }
    
    OD_Entry* entry = (OD_Entry*)malloc(sizeof(OD_Entry));
    if (entry == NULL) {
        return false;
    }
    
    entry->index = index;
    entry->subindex = subindex;
    entry->dataType = dataType;
    entry->data = NULL;  // 初始化为NULL，让OD_AddEntry来分配内存
    entry->accessRead = accessRead;
    entry->accessWrite = accessWrite;
    entry->next = NULL;
    
    bool result = OD_AddEntry(od, entry);
    if (result && data != NULL) {
        // 如果添加成功且data不为NULL，则复制数据
        size_t dataSize = OD_GetDataTypeSize(dataType);
        if (dataSize > 0) {
            memcpy(entry->data, data, dataSize);
        }
    }
    
    return result;
}

bool OD_User_Init(ObjectDictionary* od) {
    if (od == NULL) {
        return false;
    }
    
    uint8_t error_code = 0;
    uint32_t value = 0;
    
    // 使用&value的地址，但让OD_User_AddEntry内部处理数据复制
    error_code |= OD_User_AddEntry(od, OD_INDEX_ENCODER_DATA_RESET, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_PROFILE_SPEED, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_CONTROL_WORD, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_MODE_OF_OPERATION, OD_SUBINDEX_DEFAULT, OD_TYPE_INT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_POSITION, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_CURRENT, OD_SUBINDEX_DEFAULT, OD_TYPE_INT16, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_ACTUAL_VELOCITY, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, false);
    error_code |= OD_User_AddEntry(od, OD_INDEX_INVERT_DIR, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT8, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_POS, OD_SUBINDEX_DEFAULT, OD_TYPE_INT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_SPEED, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT32, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_TARGET_TORQUE, OD_SUBINDEX_DEFAULT, OD_TYPE_INT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_MAX_SPEED_RPM, OD_SUBINDEX_DEFAULT, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_SPEED_LOOP, OD_SUBINDEX_1, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_SPEED_LOOP, OD_SUBINDEX_2, OD_TYPE_UINT16, &value, true, true);
    error_code |= OD_User_AddEntry(od, OD_INDEX_GROUP_POSITION_LOOP, OD_SUBINDEX_1, OD_TYPE_INT16, &value, true, true);
    
    return error_code == 0;
}

// 删除条目
bool OD_RemoveEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex) {
    if (od == NULL || od->head == NULL) {
        return false;
    }

    OD_Entry* current = od->head;
    OD_Entry* prev = NULL;

    // 如果是头节点
    if (current->index == index && current->subindex == subindex) {
        od->head = current->next;
        if (current->data != NULL) {
            free(current->data);
        }
        free(current);
        od->entryCount--;
        return true;
    }

    // 查找要删除的节点
    while (current != NULL) {
        if (current->index == index && current->subindex == subindex) {
            prev->next = current->next;
            if (current->data != NULL) {
                free(current->data);
            }
            free(current);
            od->entryCount--;
            return true;
        }
        prev = current;
        current = current->next;
    }

    return false;
}

// 获取条目
OD_Entry* OD_GetEntry(ObjectDictionary* od, uint16_t index, uint8_t subindex) {
    return OD_FindEntry(od, index, subindex);
}

// 设置值
bool OD_SetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, void* value) {
    OD_Entry* entry = OD_FindEntry(od, index, subindex);
    if (entry == NULL || !entry->accessWrite || value == NULL) {
        return false;
    }

    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    memcpy(entry->data, value, dataSize);
    return true;
}

// 获取值
bool OD_GetValue(ObjectDictionary* od, uint16_t index, uint8_t subindex, void* value) {
    OD_Entry* entry = OD_FindEntry(od, index, subindex);
    if (entry == NULL || !entry->accessRead || value == NULL) {
        return false;
    }

    size_t dataSize = OD_GetDataTypeSize(entry->dataType);
    if (dataSize == 0) {
        return false;
    }

    memcpy(value, entry->data, dataSize);
    return true;
}

// 打印所有条目（用于调试）
void OD_PrintAll(ObjectDictionary* od) {
    if (od == NULL) {
        printf("对象字典为空\n");
        return;
    }

    // printf("对象字典内容（共%d个条目）：\n", od->entryCount);
    OD_Entry* current = od->head;
    while (current != NULL) {
        // printf("索引: 0x%04X, 子索引: %d, 类型: %d, 描述: %s\n", 
            //    current->index, current->subindex, current->dataType, 
            //    current->description ? current->description : "无描述");
        current = current->next;
    }
} 
