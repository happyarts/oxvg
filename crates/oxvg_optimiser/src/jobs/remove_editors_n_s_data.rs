use std::collections::HashSet;

use oxvg_ast::{
    element::Element,
    visitor::{Context, Visitor},
};
use oxvg_collections::{
    attribute::{Attr, AttrId},
    name::{Prefix, QualName},
};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "wasm")]
use tsify::Tsify;

use crate::error::JobsError;

#[cfg_attr(feature = "wasm", derive(Tsify))]
#[cfg_attr(feature = "napi", napi(object))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "jsonschema", derive(schemars::JsonSchema))]
#[derive(Clone, Default, Debug)]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
/// Removes all xml namespaces associated with editing software.
///
/// # Correctness
///
/// This job should never visually change the document.
///
/// Editor namespaces may be used by the editor and contain data that might be
/// lost if you try to edit the file after optimising.
///
/// # Errors
///
/// Never.
///
/// If this job produces an error or panic, please raise an [issue](https://github.com/noahbald/oxvg/issues)
pub struct RemoveEditorsNSData {
    /// A list of additional namespaces URIs you may want to remove.
    #[cfg_attr(feature = "wasm", tsify(optional))]
    pub additional_namespaces: Option<HashSet<String>>,
    /// A list of namespace URIs to keep although they are editor namespaces,
    /// e.g. RDF, Dublin Core and Creative Commons, which hold the creator and
    /// the licence.
    #[cfg_attr(feature = "wasm", tsify(optional))]
    pub keep_namespaces: Option<HashSet<String>>,
}

impl RemoveEditorsNSData {
    fn removes(&self, uri: &str) -> bool {
        let contains = |set: &Option<HashSet<String>>| set.as_ref().is_some_and(|set| set.contains(uri));
        (is_editor_namespace(uri) && !contains(&self.keep_namespaces)) || contains(&self.additional_namespaces)
    }
}

impl<'input, 'arena> Visitor<'input, 'arena> for RemoveEditorsNSData {
    type Error = JobsError<'input>;

    fn element(
        &self,
        element: Element<'input, 'arena>,
        _context: &mut Context<'input, 'arena, '_>,
    ) -> Result<(), Self::Error> {
        if self.removes(element.prefix().ns().uri()) {
            element.remove();
            return Ok(());
        }

        element.attributes().retain(|attr| {
            if let Attr::Unparsed {
                attr_id:
                    AttrId::Unknown(QualName {
                        prefix: Prefix::XMLNS,
                        local: _,
                    }),
                value,
            } = attr
            {
                return !self.removes(value);
            }

            !self.removes(attr.prefix().ns().uri())
        });

        Ok(())
    }
}

fn is_editor_namespace(uri: &str) -> bool {
    matches!(
        uri,
        "http://creativecommons.org/ns#"
            | "http://inkscape.sourceforge.net/DTD/sodipodi-0.dtd"
            | "http://ns.adobe.com/AdobeIllustrator/10.0/"
            | "http://ns.adobe.com/AdobeSVGViewerExtensions/3.0/"
            | "http://ns.adobe.com/Extensibility/1.0/"
            | "http://ns.adobe.com/Flows/1.0/"
            | "http://ns.adobe.com/GenericCustomNamespace/1.0/"
            | "http://ns.adobe.com/Graphs/1.0/"
            | "http://ns.adobe.com/ImageReplacement/1.0/"
            | "http://ns.adobe.com/SaveForWeb/1.0/"
            | "http://ns.adobe.com/Variables/1.0/"
            | "http://ns.adobe.com/XPath/1.0/"
            | "http://purl.org/dc/elements/1.1/"
            | "http://schemas.microsoft.com/visio/2003/SVGExtensions/"
            | "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
            | "http://taptrix.com/vectorillustrator/svg_extensions"
            | "http://www.bohemiancoding.com/sketch/ns"
            | "http://www.figma.com/figma/ns"
            | "http://www.inkscape.org/namespaces/inkscape"
            | "http://www.serif.com/"
            | "http://www.vector.evaxdesign.sk"
            | "http://www.w3.org/1999/02/22-rdf-syntax-ns#"
    )
}

#[test]
fn remove_editors_n_s_data() -> anyhow::Result<()> {
    use crate::test_config;

    insta::assert_snapshot!(test_config(
        r#"{ "removeEditorsNSData": {} }"#,
        Some(
            r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd">
    <sodipodi:namedview>
        ...
    </sodipodi:namedview>

    <path d="..." sodipodi:nodetypes="cccc"/>
</svg>"#
        ),
    )?);

    insta::assert_snapshot!(test_config(
        r#"{ "removeEditorsNSData": {} }"#,
        Some(
            r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:sodipodi="http://inkscape.sourceforge.net/DTD/sodipodi-0.dtd">
    <sodipodi:namedview>
        ...
    </sodipodi:namedview>

    <path d="..." sodipodi:nodetypes="cccc"/>
</svg>"#
        ),
    )?);

    insta::assert_snapshot!(test_config(
        r#"{ "removeEditorsNSData": { "keepNamespaces": ["http://www.w3.org/1999/02/22-rdf-syntax-ns#", "http://purl.org/dc/elements/1.1/", "http://creativecommons.org/ns#"] } }"#,
        Some(
            r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:cc="http://creativecommons.org/ns#" xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape">
    <metadata>
        <rdf:RDF>
            <cc:Work rdf:about="">
                <dc:creator><cc:Agent><dc:title>Jane Doe</dc:title></cc:Agent></dc:creator>
                <cc:license rdf:resource="http://creativecommons.org/licenses/by/4.0/"/>
            </cc:Work>
        </rdf:RDF>
    </metadata>
    <path d="..." inkscape:label="Layer 1"/>
</svg>"#
        ),
    )?);

    Ok(())
}
