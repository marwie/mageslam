// As described here: https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html#implement-a-c-api-in-javascript
mergeInto(LibraryManager.library, {
    mageslam_frame_processed_callback: function(isPoseGood, isPoseSkipped, trackingState, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33) {
        // Requires this handler function to be defined. Defining it is the responsibility of the consuming library.
        Module._mageslam_frame_processed_callback(isPoseGood, isPoseSkipped, trackingState, m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
    }
});
