# VRS Mutation example

`vrs_mutation` is a sample demonstrating how to create a VRS file copy with on the fly custom image modification (either from images in memory, or previously exported to disk).

## How to run the vrs_mutation example:

### Vertical flip example

This example is setting even rows to be black on all image stream (image operation processed in memory as data is loaded)
- see `NullifyEvenRowsImageMutator` in main.cpp for the implementation

`vrs_mutation -i <VRS_IN> -o <VRS_OUT> --mutationType NULLIFY_EVEN_ROWS`

### VRS image export reimport

This example allow to reimport vrs images (previously exported to disk)
- see `VrsExportLoader` in main.cpp for the implementation
- images could be modified in content (pixel value) but not in size (resolution must remain the same)

`vrs_mutation -i <VRS_IN> -o <VRS_OUT> --VRS_EXPORT_REIMPORT --exportPath <path>`

#### Notes on how to export VRS images to disk
Here is how to use this workflow (vrs binary can be compiled from [vrs repository](https://github.com/facebookresearch/vrs))
```
# 1. Export VRS Frames
$ vrs extract-images <VRS> --to /tmp/data
# 2. Modify your image frame
# - i.e blur/annotate some objects manually or with a script
#
# 3. Use VRS Copy with the `VrsExportLoader` Mutator (to replace each VRS frame with the corresponding file on disk)
vrs_mutation -i <VRS_IN> -o <VRS_OUT> --VRS_EXPORT_REIMPORT --exportPath /tmp/data
```

# How the VRS image mutation is implemented

The feature is implemented by using the `RecordFilterCopier` concept along a `vrs copy` operation.

By adding a `ImageMutationFilter` abstraction of `RecordFilterCopier` with a custom `UserDefinedImageMutator`, the user can modify on the fly the PixelFrame image read from a given VRS and export it to a new VRS file.
FYI:
- Image size and bit depth must remains the same.
- `ImageMutationFilter::shouldCopyVerbatim` implements the logic to apply the functor only on image stream
- `ImageMutationFilter::filterImage` implements the JPG buffer codec logic and allow you to access the uncompressed PixelFrame for mutation with your functor

### How to write your how custom image mutation?
- See the provided examples `NullifyEvenRowsImageMutator`, or extend the `VrsExportLoader`.

```
struct MyCustomImageMutator : public vrs::utils::UserDefinedImageMutator {
  bool operator()(double timestamp, const vrs::StreamId& streamId, vrs::utils::PixelFrame* frame)
      override {
    if (!frame) { // If frame is invalid, Do nothing.
      return false;
    }
    // If frame is valid:
    // - apply your image processing function
    // - or load an image from disk to replace the image buffer
    // -> Note the image size (Width, Height, Stride) must be left unchanged

    // Image is defined by its PixelFrame:
    // frame->getWidth()
    // frame->getHeight()
    //
    // frame->wdata() - Pixel image buffer start (of size frame->getStride()*frame->getHeight() )
    //
    // You could iterate the image from top to bottom as following:
    //
    // const size_t lineLength = frame->getStride();
    // uint32_t top = 0;
    // uint32_t bottom = frame->getHeight() - 1;
    // while (top < bottom) {
    //   uint8_t* topPixels = frame->wdata() + top * frame->getStride();
    //   top++;
    // }

    return true;
  }
};
```
