#include <FoxglovePointCloudUtilities.hpp>
#include <type_traits>

using namespace pcl;



void addPointFieldToFieldArray(
  google::protobuf::RepeatedPtrField<foxglove::PackedElementField>* pointFieldArray,
  const std::string& name,
  int& offset,
  foxglove::PackedElementField_NumericType type
)
{
  auto pointField = pointFieldArray->Add();
  pointField->set_name(name);
  pointField->set_offset(offset);
  pointField->set_type(type);

  auto datatypeSize = FoxgloveDatatypeSizesMap.at(type);
  offset += datatypeSize;
}