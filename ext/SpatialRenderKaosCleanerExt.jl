# AttributeOutlierFilter backed by the KaosCleaner statistics engine.
module SpatialRenderKaosCleanerExt

using SpatialRender
using SpatialRender: AttributeOutlierFilter, PointCloud, attribute_column
import KaosCleaner

const METHODS = ("zscore", "iqr", "mad", "hampel")

"""
Keep-mask for `AttributeOutlierFilter`: a point survives when none of the
selected attribute columns flags it. Hampel windows follow GPS time when the
cloud has it (scan order otherwise).
"""
function attribute_keep_mask(f::AttributeOutlierFilter, pc::PointCloud)
    f.method in METHODS || throw(ArgumentError("unknown outlier method $(repr(f.method)); use one of $(join(METHODS, ", "))"))
    k = something(f.threshold, KaosCleaner.OUTLIER_DEFAULTS[f.method])
    times = isempty(pc.gps_time) ? nothing : pc.gps_time
    keep = trues(length(pc))
    for a in f.attributes
        flagged = KaosCleaner.outlier_mask(attribute_column(pc, a), f.method, k, f.window, times)
        keep .&= .!flagged
    end
    return keep
end

end
