import UrduTranslatorAgent from './translator-agent.js';

// Initialize the translator agent
const translator = new UrduTranslatorAgent({
  apiUrl: 'https://api.example.com/translate',
  apiKey: process.env.TRANSLATOR_API_KEY || 'YOUR_API_KEY_HERE'
});

// Example: Basic translation to Urdu
async function basicTranslation() {
  try {
    console.log('Performing basic translation...');
    const result = await translator.translate('Hello, how are you?', 'ur');
    console.log('Original:', result.originalText);
    console.log('Translated:', result.translatedText);
    console.log('Success:', result.success);
  } catch (error) {
    console.error('Translation failed:', error.message);
  }
}

// Example: Translation with legal context
async function legalTranslation() {
  try {
    console.log('\nPerforming legal context translation...');
    const legalText = 'Fundamental rights and constitutional provisions';
    const result = await translator.translateWithContext(legalText, 'ur', { type: 'legal' });
    console.log('Original:', result.originalText);
    console.log('Translated:', result.translatedText);
    console.log('Context preserved:', result.contextPreserved);
  } catch (error) {
    console.error('Legal translation failed:', error.message);
  }
}

// Example: Translation with educational context
async function educationalTranslation() {
  try {
    console.log('\nPerforming educational context translation...');
    const educationText = 'Education is the foundation of a progressive society';
    const result = await translator.translateWithContext(educationText, 'ur', { type: 'educational' });
    console.log('Original:', result.originalText);
    console.log('Translated:', result.translatedText);
    console.log('Context preserved:', result.contextPreserved);
  } catch (error) {
    console.error('Educational translation failed:', error.message);
  }
}

// Example: Batch translation
async function batchTranslation() {
  try {
    console.log('\nPerforming batch translation...');
    const texts = [
      'Hello world',
      'How are you?',
      'Thank you',
      'Have a nice day'
    ];
    const results = await translator.batchTranslate(texts, 'ur');

    results.forEach((result, index) => {
      if (result.success) {
        console.log(`Text ${index + 1}: ${result.originalText} -> ${result.translatedText}`);
      } else {
        console.log(`Text ${index + 1} failed: ${result.error}`);
      }
    });
  } catch (error) {
    console.error('Batch translation failed:', error.message);
  }
}

// Example: Checking constitutional compliance
async function complianceCheck() {
  try {
    console.log('\nTesting constitutional compliance...');

    // This should work fine
    const compliantText = 'Respect for diversity and equality';
    const result1 = await translator.translate(compliantText, 'ur');
    console.log('Compliant translation successful:', result1.translatedText);

    // This might fail depending on the content filters
    const potentiallyNonCompliant = 'Hateful content that violates principles'; // This is just an example
    try {
      const result2 = await translator.translate(potentiallyNonCompliant, 'ur');
      console.log('Potentially non-compliant translation:', result2.translatedText);
    } catch (error) {
      console.log('Translation correctly rejected non-compliant content:', error.message);
    }
  } catch (error) {
    console.error('Compliance check failed:', error.message);
  }
}

// Run all examples
async function runAllExamples() {
  await basicTranslation();
  await legalTranslation();
  await educationalTranslation();
  await batchTranslation();
  await complianceCheck();

  console.log('\nAll examples completed!');
}

// Uncomment the next line to run the examples
// runAllExamples();

// Export for use in other modules
export {
  translator,
  basicTranslation,
  legalTranslation,
  educationalTranslation,
  batchTranslation,
  complianceCheck,
  runAllExamples
};